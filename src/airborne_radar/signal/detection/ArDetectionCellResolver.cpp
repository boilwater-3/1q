#include "airborne_radar/signal/detection/ArDetectionCellResolver.h"

#include <algorithm>
#include <cmath>
#include <tuple>
#include <utility>

namespace airborne_radar {
namespace signal {
namespace detection {
namespace {

constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kBoltzmannJPerK = 1.380649e-23;
constexpr double kPi = 3.14159265358979323846;
constexpr double kLinearFloor = 1.0e-300;
constexpr std::uint32_t kRepresentativePulseLimit = 8U;
constexpr std::uint32_t kSamplesPerEchoGate = 5U;
/** @brief 增益偏置值域（与配置校验一致；求解器内钳位兜底防越界指数）。 */
constexpr double kMaxGainOffsetDb = 40.0;

bool IsFinitePositive(double value) { return std::isfinite(value) && value > 0.0; }

/** @brief dB 偏置 → 线性因子，[0, 40] dB 钳位。 */
double GainFactorFromDb(double gain_db) {
  const double clamped = std::min(std::max(gain_db, 0.0), kMaxGainOffsetDb);
  return std::pow(10.0, clamped / 10.0);
}

bool SameIdentity(const oneq::electromagnetics::RfEmissionIdentity& left,
                  const oneq::electromagnetics::RfEmissionIdentity& right) {
  return left.platform_id == right.platform_id && left.equipment_id == right.equipment_id &&
         left.emission_id == right.emission_id;
}

bool IsValidIdentity(const oneq::electromagnetics::RfEmissionIdentity& identity) {
  return identity.platform_id != 0U && identity.equipment_id != 0U &&
         identity.emission_id != 0U;
}

double IntervalOverlap(double left_center_hz, double left_bandwidth_hz,
                       double right_center_hz, double right_bandwidth_hz) {
  const double left_low = left_center_hz - 0.5 * left_bandwidth_hz;
  const double left_high = left_center_hz + 0.5 * left_bandwidth_hz;
  const double right_low = right_center_hz - 0.5 * right_bandwidth_hz;
  const double right_high = right_center_hz + 0.5 * right_bandwidth_hz;
  return std::max(0.0, std::min(left_high, right_high) - std::max(left_low, right_low));
}

bool TryCountAvailableEchoPulses(const oneq::electromagnetics::RfWaveformSchedule& waveform,
                                 double echo_delay_s, double receive_window_end_s,
                                 std::uint32_t requested_count, std::uint32_t* available_count) {
  if (available_count == nullptr) {
    return false;
  }
  std::uint32_t low = 0U;
  std::uint32_t high = std::min(requested_count, waveform.pulse_count);
  while (low < high) {
    const std::uint32_t middle = low + (high - low) / 2U;
    double pulse_start_s = 0.0;
    if (!oneq::electromagnetics::TryResolveRfPulseStartTime(waveform, middle,
                                                            &pulse_start_s)) {
      return false;
    }
    if (pulse_start_s + echo_delay_s < receive_window_end_s) {
      low = middle + 1U;
    } else {
      high = middle;
    }
  }
  *available_count = low;
  return true;
}

bool TryResolveCellInterference(
    const ArDetectionCellConfig& config, double echo_delay_s, double target_cell_center_hz,
    std::uint32_t effective_pulse_count,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    double* interference_power_w) {
  if (interference_power_w == nullptr) {
    return false;
  }
  typedef std::tuple<std::uint64_t, std::uint64_t, std::uint64_t> IdentityKey;
  std::vector<std::pair<IdentityKey, const oneq::electromagnetics::RfIncidentLinkResult*> > ordered;
  ordered.reserve(incident_links.size());
  for (const auto& link : incident_links) {
    if (!IsValidIdentity(link.identity) || !std::isfinite(link.received_power_before_overlap_w) ||
        link.received_power_before_overlap_w < 0.0 || !std::isfinite(link.propagation_delay_s) ||
        link.propagation_delay_s < 0.0 || !std::isfinite(link.doppler_shift_hz) ||
        !IsFinitePositive(link.emission_waveform.occupied_bandwidth_hz)) {
      return false;
    }
    ordered.push_back(std::make_pair(
        std::make_tuple(link.identity.platform_id, link.identity.equipment_id,
                        link.identity.emission_id),
        &link));
  }
  std::sort(ordered.begin(), ordered.end(),
            [](const std::pair<IdentityKey,
                               const oneq::electromagnetics::RfIncidentLinkResult*>& left,
               const std::pair<IdentityKey,
                               const oneq::electromagnetics::RfIncidentLinkResult*>& right) {
              return left.first < right.first;
            });
  for (std::size_t index = 1U; index < ordered.size(); ++index) {
    if (ordered[index - 1U].first == ordered[index].first) {
      return false;
    }
  }

  if (effective_pulse_count == 0U) {
    *interference_power_w = 0.0;
    return true;
  }
  const std::uint32_t representative_count =
      std::min(effective_pulse_count, kRepresentativePulseLimit);
  long double total_interference_w = 0.0L;
  for (const auto& ordered_link : ordered) {
    const auto& link = *ordered_link.second;
    if (SameIdentity(link.identity, own_emission_identity)) {
      continue;
    }
    long double link_overlap_sum = 0.0L;
    for (std::uint32_t representative = 0U; representative < representative_count;
         ++representative) {
      const std::uint32_t pulse_index =
          representative_count == 1U
              ? 0U
              : static_cast<std::uint32_t>(
                    (static_cast<std::uint64_t>(representative) *
                     static_cast<std::uint64_t>(effective_pulse_count - 1U)) /
                    static_cast<std::uint64_t>(representative_count - 1U));
      double pulse_start_s = 0.0;
      if (!oneq::electromagnetics::TryResolveRfPulseStartTime(
              config.own_transmit_waveform, pulse_index, &pulse_start_s)) {
        return false;
      }
      for (std::uint32_t sample = 0U; sample < kSamplesPerEchoGate; ++sample) {
        const double sample_fraction =
            (static_cast<double>(sample) + 0.5) / static_cast<double>(kSamplesPerEchoGate);
        const double arrival_time_s = pulse_start_s + echo_delay_s +
                                      sample_fraction * config.own_transmit_waveform.pulse_width_s;
        bool active = false;
        double arrival_center_frequency_hz = 0.0;
        if (!oneq::electromagnetics::TryEvaluateRfArrivalActivity(
                link.emission_waveform, link.propagation_delay_s, link.doppler_shift_hz,
                arrival_time_s, &active, &arrival_center_frequency_hz)) {
          return false;
        }
        if (!active) {
          continue;
        }
        const double overlap_hz =
            IntervalOverlap(target_cell_center_hz, config.matched_filter_bandwidth_hz,
                            arrival_center_frequency_hz,
                            link.emission_waveform.occupied_bandwidth_hz);
        link_overlap_sum += overlap_hz / link.emission_waveform.occupied_bandwidth_hz;
      }
    }
    const long double sample_count = static_cast<long double>(representative_count) *
                                     static_cast<long double>(kSamplesPerEchoGate);
    long double effective_power = static_cast<long double>(link.received_power_before_overlap_w);
    if (config.enable_anti_rgpo_leading_edge &&
        link.emission_waveform.kind ==
            oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain) {
      effective_power *= 0.5L;
    }
    total_interference_w += effective_power * link_overlap_sum / sample_count;
  }
  const double candidate = static_cast<double>(total_interference_w);
  if (!std::isfinite(candidate) || candidate < 0.0) {
    return false;
  }
  *interference_power_w = candidate;
  return true;
}

bool AreGainsUsable(const config::detection::SignalProcessingConfig& gains) {
  return std::isfinite(gains.target_processing_gain_db) &&
         std::isfinite(gains.noise_processing_gain_db) &&
         std::isfinite(gains.clutter_suppression_gain_db) &&
         std::isfinite(gains.jamming_suppression_gain_db);
}

}  // namespace

bool TryResolveArDetectionCell(
    const ArDetectionCellConfig& config, const ArDetectionCellTarget& target,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    double clutter_power_w, ArDetectionCellResult* result) {
  const auto& own_waveform = config.own_transmit_waveform;
  if (result == nullptr || !IsValidIdentity(own_emission_identity) ||
      own_waveform.kind != oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain ||
      !IsFinitePositive(config.receive_window_duration_s) ||
      !std::isfinite(config.receive_window_start_time_s) ||
      !IsFinitePositive(config.matched_filter_bandwidth_hz) ||
      !std::isfinite(config.one_way_antenna_gain_dbi) || !std::isfinite(config.receiver_loss_db) ||
      config.receiver_loss_db < 0.0 || !std::isfinite(config.receiver_noise_figure_db) ||
      config.receiver_noise_figure_db < 0.0 || !IsFinitePositive(config.reference_temperature_k) ||
      !AreGainsUsable(config.signal_processing) ||
      !IsFinitePositive(target.range_m) || !std::isfinite(target.closing_radial_velocity_mps) ||
      !IsFinitePositive(target.rcs_m2) ||
      !std::isfinite(target.two_way_additional_propagation_loss_db) ||
      target.two_way_additional_propagation_loss_db < 0.0 || target.effective_pulse_count == 0U ||
      !std::isfinite(clutter_power_w) || clutter_power_w < 0.0) {
    return false;
  }
  double first_pulse_start_s = 0.0;
  if (!oneq::electromagnetics::TryResolveRfPulseStartTime(own_waveform, 0U,
                                                          &first_pulse_start_s) ||
      first_pulse_start_s < config.receive_window_start_time_s ||
      !IsFinitePositive(own_waveform.center_frequency_hz) ||
      !IsFinitePositive(own_waveform.pulse_width_s) ||
      !IsFinitePositive(own_waveform.transmit_power_w)) {
    return false;
  }

  ArDetectionCellResult candidate;
  const double wavelength_m = kSpeedOfLightMps / own_waveform.center_frequency_hz;
  const double antenna_gain_linear = std::pow(10.0, config.one_way_antenna_gain_dbi / 10.0);
  const double total_loss_linear = std::pow(
      10.0, (config.receiver_loss_db + target.two_way_additional_propagation_loss_db) / 10.0);
  const double range_squared_m2 = target.range_m * target.range_m;
  const double range_fourth_m4 = range_squared_m2 * range_squared_m2;
  const double geometric_denominator = std::pow(4.0 * kPi, 3.0) * range_fourth_m4;
  candidate.echo_power_w = own_waveform.transmit_power_w * antenna_gain_linear *
                           antenna_gain_linear * wavelength_m * wavelength_m * target.rcs_m2 /
                           (geometric_denominator * total_loss_linear);
  candidate.echo_delay_s = 2.0 * target.range_m / kSpeedOfLightMps;
  candidate.two_way_doppler_shift_hz =
      2.0 * own_waveform.center_frequency_hz * target.closing_radial_velocity_mps /
      kSpeedOfLightMps;
  candidate.pulse_compression_gain =
      std::max(1.0, config.matched_filter_bandwidth_hz * own_waveform.pulse_width_s);
  const double noise_figure_linear = std::pow(10.0, config.receiver_noise_figure_db / 10.0);
  candidate.thermal_noise_power_w = kBoltzmannJPerK * config.reference_temperature_k *
                                    config.matched_filter_bandwidth_hz * noise_figure_linear;
  if (!TryCountAvailableEchoPulses(
          own_waveform, candidate.echo_delay_s,
          config.receive_window_start_time_s + config.receive_window_duration_s,
          target.effective_pulse_count, &candidate.effective_pulse_count) ||
      !TryResolveCellInterference(
          config, candidate.echo_delay_s,
          own_waveform.center_frequency_hz + candidate.two_way_doppler_shift_hz,
          candidate.effective_pulse_count, own_emission_identity, incident_links,
          &candidate.interference_power_w)) {
    return false;
  }
  candidate.clutter_power_w = clutter_power_w;
  // 分项 SINR 账本 + 四增益偏置：
  // 分子 = 回波 × 脉压 × target 偏置；分母 = 热噪声 × noise 偏置
  //        + 干扰 ÷ jamming 抑制 + 杂波 ÷ clutter 抑制（保守口径不加脉压）。
  const double target_gain_linear = GainFactorFromDb(
      static_cast<double>(config.signal_processing.target_processing_gain_db));
  const double noise_gain_linear = GainFactorFromDb(
      static_cast<double>(config.signal_processing.noise_processing_gain_db));
  const double jamming_suppression_linear = GainFactorFromDb(
      static_cast<double>(config.signal_processing.jamming_suppression_gain_db));
  const double clutter_suppression_linear = GainFactorFromDb(
      static_cast<double>(config.signal_processing.clutter_suppression_gain_db));
  const double denominator = std::max(
      candidate.thermal_noise_power_w * noise_gain_linear +
          candidate.interference_power_w / jamming_suppression_linear +
          candidate.clutter_power_w / clutter_suppression_linear,
      kLinearFloor);
  candidate.processed_single_pulse_sinr_linear =
      candidate.echo_power_w * candidate.pulse_compression_gain * target_gain_linear /
      denominator;
  candidate.processed_single_pulse_sinr_db =
      10.0 * std::log10(std::max(candidate.processed_single_pulse_sinr_linear, kLinearFloor));
  if (!std::isfinite(candidate.echo_power_w) || candidate.echo_power_w < 0.0 ||
      !std::isfinite(candidate.processed_single_pulse_sinr_db)) {
    return false;
  }
  *result = candidate;
  return true;
}

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar
