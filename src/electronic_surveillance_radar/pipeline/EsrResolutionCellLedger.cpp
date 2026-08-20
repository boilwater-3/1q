#include "electronic_surveillance_radar/pipeline/EsrResolutionCellLedger.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace electronic_surveillance_radar {
namespace pipeline {
namespace {

constexpr std::size_t kTimeCellCount = 256U;

struct CellContribution {
  std::size_t source_index{0U};
  double lower_frequency_hz{0.0};
  double upper_frequency_hz{0.0};
  double center_frequency_hz{0.0};
  double average_power_w{0.0};
  double active_time_s{0.0};
  std::vector<std::uint32_t> pulse_indices{};
  bool candidate_eligible{false};
};

using AngularCellKey = std::pair<std::int32_t, std::int32_t>;

struct SourceAccumulator {
  double signal_energy_j{0.0};
  double interference_energy_j{0.0};
  double frequency_weighted_energy_j_hz{0.0};
  double active_time_s{0.0};
  std::set<std::uint32_t> pulse_indices{};
};

bool IsFinite(double value) { return std::isfinite(value) != 0; }

double IntervalOverlap(double left_start, double left_end, double right_start,
                       double right_end) {
  return std::max(0.0, std::min(left_end, right_end) -
                           std::max(left_start, right_start));
}

double FrequencyOverlapFraction(double source_center_hz, double source_bandwidth_hz,
                                const oneq::electromagnetics::RfSceneReceiverState& receiver) {
  const double source_lower = source_center_hz - 0.5 * source_bandwidth_hz;
  const double source_upper = source_center_hz + 0.5 * source_bandwidth_hz;
  const double receiver_lower = receiver.center_frequency_hz - 0.5 * receiver.bandwidth_hz;
  const double receiver_upper = receiver.center_frequency_hz + 0.5 * receiver.bandwidth_hz;
  const double overlap = std::max(
      0.0, std::min(source_upper, receiver_upper) -
               std::max(source_lower, receiver_lower));
  return std::max(0.0, std::min(1.0, overlap / source_bandwidth_hz));
}

AngularCellKey ResolveAngularCell(const EsrArrivalBearing& bearing,
                                  double angular_resolution_deg) {
  const double azimuth = std::max(-180.0, std::min(180.0, bearing.azimuth_deg));
  const double elevation = std::max(-90.0, std::min(90.0, bearing.elevation_deg));
  return std::make_pair(
      static_cast<std::int32_t>(std::floor((azimuth + 180.0) / angular_resolution_deg)),
      static_cast<std::int32_t>(std::floor((elevation + 90.0) / angular_resolution_deg)));
}

bool AddContributionForBin(std::size_t source_index,
                           const oneq::electromagnetics::RfIncidentLinkResult& link,
                           const EsrArrivalBearing& bearing,
                           const oneq::electromagnetics::RfSceneReceiverState& receiver,
                           double angular_resolution_deg, std::size_t time_bin,
                           double active_time_s, const std::vector<std::uint32_t>& pulse_indices,
                           double arrival_sample_time_s,
                           std::map<AngularCellKey, std::vector<CellContribution>>* angular_cells,
                           std::vector<double>* co_site_power_w) {
  if (active_time_s <= 0.0 || angular_cells == nullptr || co_site_power_w == nullptr) {
    return true;
  }
  bool active = false;
  double center_frequency_hz = 0.0;
  if (!oneq::electromagnetics::TryEvaluateRfArrivalActivity(
          link.emission_waveform, link.propagation_delay_s, link.doppler_shift_hz,
          arrival_sample_time_s, &active, &center_frequency_hz)) {
    return false;
  }
  if (!active) {
    // Pulse bins use an exact overlap duration; their midpoint may fall
    // outside the pulse when several narrow pulses share one coarse bin.
    if (link.emission_waveform.kind !=
        oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain) {
      return true;
    }
    center_frequency_hz =
        link.emission_waveform.center_frequency_hz + link.doppler_shift_hz;
  }
  const double bandwidth_hz = link.emission_waveform.occupied_bandwidth_hz;
  const double overlap_fraction =
      FrequencyOverlapFraction(center_frequency_hz, bandwidth_hz, receiver);
  if (overlap_fraction <= 0.0) {
    return true;
  }
  const double time_bin_duration_s =
      receiver.window_duration_s / static_cast<double>(kTimeCellCount);
  const double bounded_active_time_s = std::min(time_bin_duration_s, active_time_s);
  const double average_power_w = link.received_power_before_overlap_w * overlap_fraction *
                                 (bounded_active_time_s / time_bin_duration_s);
  if (!IsFinite(average_power_w) || average_power_w < 0.0) {
    return false;
  }
  if (link.is_co_site || !bearing.defined) {
    (*co_site_power_w)[time_bin] += average_power_w;
    return IsFinite((*co_site_power_w)[time_bin]);
  }
  CellContribution contribution;
  contribution.source_index = source_index;
  contribution.lower_frequency_hz = center_frequency_hz - 0.5 * bandwidth_hz;
  contribution.upper_frequency_hz = center_frequency_hz + 0.5 * bandwidth_hz;
  contribution.center_frequency_hz = center_frequency_hz;
  contribution.average_power_w = average_power_w;
  contribution.active_time_s = bounded_active_time_s;
  contribution.pulse_indices = pulse_indices;
  contribution.candidate_eligible = bearing.azimuth_observable;
  (*angular_cells)[ResolveAngularCell(bearing, angular_resolution_deg)].push_back(contribution);
  return true;
}

bool AccumulateLink(
    std::size_t source_index,
    const oneq::electromagnetics::RfIncidentLinkResult& link,
    const EsrArrivalBearing& bearing,
    const oneq::electromagnetics::RfSceneReceiverState& receiver,
    double angular_resolution_deg,
    std::vector<std::map<AngularCellKey, std::vector<CellContribution>>>* time_cells,
    std::vector<double>* co_site_power_w) {
  const double receiver_start_s = receiver.window_start_time_s;
  const double time_bin_duration_s =
      receiver.window_duration_s / static_cast<double>(kTimeCellCount);
  std::vector<double> active_time_by_bin(kTimeCellCount, 0.0);
  std::vector<std::vector<std::uint32_t>> pulse_indices_by_bin(kTimeCellCount);
  const oneq::electromagnetics::RfWaveformSchedule& waveform = link.emission_waveform;
  if (waveform.kind == oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain) {
    for (std::uint32_t pulse_index = 0U; pulse_index < waveform.pulse_count; ++pulse_index) {
      double pulse_start_s = 0.0;
      if (!oneq::electromagnetics::TryResolveRfPulseStartTime(
              waveform, pulse_index, &pulse_start_s)) {
        return false;
      }
      pulse_start_s += link.propagation_delay_s;
      const double pulse_end_s = pulse_start_s + waveform.pulse_width_s;
      const double clipped_start_s = std::max(pulse_start_s, receiver_start_s);
      const double clipped_end_s =
          std::min(pulse_end_s, receiver_start_s + receiver.window_duration_s);
      if (clipped_end_s <= clipped_start_s) {
        continue;
      }
      const std::size_t first_bin = std::min(
          kTimeCellCount - 1U,
          static_cast<std::size_t>((clipped_start_s - receiver_start_s) /
                                   time_bin_duration_s));
      const std::size_t last_bin = std::min(
          kTimeCellCount - 1U,
          static_cast<std::size_t>(
              std::nextafter(clipped_end_s - receiver_start_s, 0.0) /
              time_bin_duration_s));
      for (std::size_t bin = first_bin; bin <= last_bin; ++bin) {
        const double bin_start_s =
            receiver_start_s + static_cast<double>(bin) * time_bin_duration_s;
        const double overlap_s = IntervalOverlap(
            clipped_start_s, clipped_end_s, bin_start_s,
            bin_start_s + time_bin_duration_s);
        if (overlap_s > 0.0) {
          active_time_by_bin[bin] += overlap_s;
          pulse_indices_by_bin[bin].push_back(pulse_index);
        }
      }
    }
  } else {
    const double arrival_start_s =
        waveform.activity_start_time_s + link.propagation_delay_s;
    const double arrival_end_s = arrival_start_s + waveform.activity_duration_s;
    for (std::size_t bin = 0U; bin < kTimeCellCount; ++bin) {
      const double bin_start_s =
          receiver_start_s + static_cast<double>(bin) * time_bin_duration_s;
      active_time_by_bin[bin] =
          IntervalOverlap(arrival_start_s, arrival_end_s, bin_start_s,
                          bin_start_s + time_bin_duration_s);
    }
  }

  for (std::size_t bin = 0U; bin < kTimeCellCount; ++bin) {
    if (active_time_by_bin[bin] <= 0.0) {
      continue;
    }
    const double sample_time_s =
        receiver_start_s + (static_cast<double>(bin) + 0.5) * time_bin_duration_s;
    if (!AddContributionForBin(source_index, link, bearing, receiver, angular_resolution_deg, bin,
                               active_time_by_bin[bin], pulse_indices_by_bin[bin], sample_time_s,
                               &(*time_cells)[bin], co_site_power_w)) {
      return false;
    }
  }
  return true;
}

void AccumulateFrequencyClusters(
    std::vector<CellContribution>* contributions, double co_site_power_w,
    double time_bin_duration_s, std::vector<SourceAccumulator>* accumulators) {
  std::sort(contributions->begin(), contributions->end(),
            [](const CellContribution& left, const CellContribution& right) {
              return std::tie(left.lower_frequency_hz, left.upper_frequency_hz,
                              left.source_index) <
                     std::tie(right.lower_frequency_hz, right.upper_frequency_hz,
                              right.source_index);
            });
  std::size_t begin = 0U;
  while (begin < contributions->size()) {
    std::size_t end = begin + 1U;
    double cluster_upper = (*contributions)[begin].upper_frequency_hz;
    while (end < contributions->size() &&
           (*contributions)[end].lower_frequency_hz < cluster_upper) {
      cluster_upper = std::max(cluster_upper, (*contributions)[end].upper_frequency_hz);
      ++end;
    }
    std::size_t strongest = end;
    for (std::size_t index = begin; index < end; ++index) {
      if ((*contributions)[index].candidate_eligible &&
          (strongest == end ||
           (*contributions)[index].average_power_w >
               (*contributions)[strongest].average_power_w ||
           ((*contributions)[index].average_power_w ==
                (*contributions)[strongest].average_power_w &&
            (*contributions)[index].source_index <
                (*contributions)[strongest].source_index))) {
        strongest = index;
      }
    }
    if (strongest == end) {
      begin = end;
      continue;
    }
    const CellContribution& candidate = (*contributions)[strongest];
    SourceAccumulator& accumulator = (*accumulators)[candidate.source_index];
    const double signal_energy_j = candidate.average_power_w * time_bin_duration_s;
    double interference_energy_j = co_site_power_w * candidate.active_time_s;
    for (std::size_t index = begin; index < end; ++index) {
      if (index == strongest || (*contributions)[index].active_time_s <= 0.0) {
        continue;
      }
      const CellContribution& interferer = (*contributions)[index];
      const double instantaneous_power_w =
          interferer.average_power_w * time_bin_duration_s / interferer.active_time_s;
      interference_energy_j +=
          instantaneous_power_w * std::min(candidate.active_time_s, interferer.active_time_s);
    }
    accumulator.signal_energy_j += signal_energy_j;
    accumulator.interference_energy_j += interference_energy_j;
    accumulator.frequency_weighted_energy_j_hz +=
        signal_energy_j * candidate.center_frequency_hz;
    accumulator.active_time_s += candidate.active_time_s;
    accumulator.pulse_indices.insert(candidate.pulse_indices.begin(),
                                     candidate.pulse_indices.end());
    begin = end;
  }
}

}  // namespace

bool TryBuildEsrResolutionCellLedger(
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    const std::vector<EsrArrivalBearing>& bearings,
    const oneq::electromagnetics::RfSceneReceiverState& receiver,
    double angular_resolution_deg, EsrResolutionCellLedgerResult* result) {
  if (result == nullptr || incident_links.size() != bearings.size() ||
      !IsFinite(receiver.window_start_time_s) ||
      !IsFinite(receiver.window_duration_s) || receiver.window_duration_s <= 0.0 ||
      !IsFinite(receiver.center_frequency_hz) ||
      receiver.center_frequency_hz <= 0.0 || !IsFinite(receiver.bandwidth_hz) ||
      receiver.bandwidth_hz <= 0.0 || !IsFinite(angular_resolution_deg) ||
      angular_resolution_deg <= 0.0) {
    return false;
  }
  std::vector<std::map<AngularCellKey, std::vector<CellContribution>>> time_cells(
      kTimeCellCount);
  std::vector<double> co_site_power_w(kTimeCellCount, 0.0);
  for (std::size_t source_index = 0U; source_index < incident_links.size();
       ++source_index) {
    if (!AccumulateLink(source_index, incident_links[source_index],
                        bearings[source_index], receiver, angular_resolution_deg,
                        &time_cells, &co_site_power_w)) {
      return false;
    }
  }

  const double time_bin_duration_s =
      receiver.window_duration_s / static_cast<double>(kTimeCellCount);
  std::vector<SourceAccumulator> accumulators(incident_links.size());
  for (std::size_t bin = 0U; bin < time_cells.size(); ++bin) {
    for (auto& angular_cell : time_cells[bin]) {
      AccumulateFrequencyClusters(&angular_cell.second, co_site_power_w[bin],
                                  time_bin_duration_s, &accumulators);
    }
  }

  EsrResolutionCellLedgerResult candidate_result;
  for (std::size_t source_index = 0U; source_index < accumulators.size();
       ++source_index) {
    const SourceAccumulator& accumulator = accumulators[source_index];
    if (accumulator.signal_energy_j <= 0.0 || accumulator.active_time_s <= 0.0) {
      continue;
    }
    EsrResolutionCellCandidate candidate;
    candidate.source_index = source_index;
    candidate.signal_power_w = accumulator.signal_energy_j / accumulator.active_time_s;
    candidate.interference_power_w = accumulator.interference_energy_j / accumulator.active_time_s;
    candidate.estimated_center_frequency_hz =
        accumulator.frequency_weighted_energy_j_hz /
        accumulator.signal_energy_j;
    candidate.active_time_s =
        std::min(receiver.window_duration_s, accumulator.active_time_s);
    candidate.effective_pulse_count = static_cast<std::uint32_t>(std::max<std::size_t>(
        1U, std::min<std::size_t>(accumulator.pulse_indices.size(),
                                  std::numeric_limits<std::uint32_t>::max())));
    if (!IsFinite(candidate.signal_power_w) ||
        !IsFinite(candidate.interference_power_w) ||
        !IsFinite(candidate.estimated_center_frequency_hz) ||
        !IsFinite(candidate.active_time_s)) {
      return false;
    }
    candidate_result.candidates.push_back(candidate);
  }
  *result = std::move(candidate_result);
  return true;
}

}  // namespace pipeline
}  // namespace electronic_surveillance_radar
