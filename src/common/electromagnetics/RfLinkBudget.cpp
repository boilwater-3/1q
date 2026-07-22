// Copyright 2026. All Rights Reserved.

#include "1q/electromagnetics/RfLinkBudget.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

namespace oneq {
namespace electromagnetics {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kLinearCircularLossDb = 3.010299956639812;

bool IsFinite(double value) { return std::isfinite(value); }

bool IsFinitePosition(const coordinate::EcefPositionM& position) {
  return IsFinite(position.x_m) && IsFinite(position.y_m) && IsFinite(position.z_m);
}

bool IsFiniteVelocity(const coordinate::EcefVelocityMps& velocity) {
  return IsFinite(velocity.x_mps) && IsFinite(velocity.y_mps) && IsFinite(velocity.z_mps);
}

bool IsFiniteVector(const RfEcefUnitVector& vector) {
  return IsFinite(vector.x) && IsFinite(vector.y) && IsFinite(vector.z);
}

bool IsKnownPolarization(RfPolarization polarization) {
  switch (polarization) {
    case RfPolarization::kHorizontal:
    case RfPolarization::kVertical:
    case RfPolarization::kRightHandCircular:
    case RfPolarization::kLeftHandCircular:
    case RfPolarization::kUnpolarized:
      return true;
  }
  return false;
}

bool IsKnownWaveform(RfWaveformKind waveform) {
  switch (waveform) {
    case RfWaveformKind::kContinuous:
    case RfWaveformKind::kPulsed:
    case RfWaveformKind::kSwept:
    case RfWaveformKind::kNoise:
      return true;
  }
  return false;
}

bool TryNormalize(const RfEcefUnitVector& vector, RfEcefUnitVector* normalized) {
  if (normalized == nullptr || !IsFiniteVector(vector)) {
    return false;
  }
  const double norm_squared = vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;
  if (!IsFinite(norm_squared) || norm_squared <= 0.0) {
    return false;
  }
  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  RfEcefUnitVector candidate;
  candidate.x = vector.x * inverse_norm;
  candidate.y = vector.y * inverse_norm;
  candidate.z = vector.z * inverse_norm;
  if (!IsFiniteVector(candidate)) {
    return false;
  }
  *normalized = candidate;
  return true;
}

bool IsValidPattern(const RfAntennaPattern& pattern) {
  RfEcefUnitVector normalized;
  return TryNormalize(pattern.boresight_ecef_unit, &normalized) &&
         IsFinite(pattern.peak_gain_dbi) && IsFinite(pattern.half_power_beamwidth_deg) &&
         pattern.half_power_beamwidth_deg > 0.0 && pattern.half_power_beamwidth_deg <= 180.0 &&
         IsFinite(pattern.sidelobe_level_db) && pattern.sidelobe_level_db <= -3.0 &&
         IsFinite(pattern.backlobe_level_db) &&
         pattern.backlobe_level_db <= pattern.sidelobe_level_db &&
         IsFinite(pattern.cross_polarization_isolation_db) &&
         pattern.cross_polarization_isolation_db >= 0.0;
}

/** @brief 按轴对称 3 dB 主瓣和旁后瓣平台计算指定方向增益。 */
bool TryEvaluateGain(const RfAntennaPattern& pattern, const RfEcefUnitVector& look_ecef,
                     double* gain_dbi) {
  if (gain_dbi == nullptr || !IsValidPattern(pattern)) {
    return false;
  }
  RfEcefUnitVector boresight;
  RfEcefUnitVector look;
  if (!TryNormalize(pattern.boresight_ecef_unit, &boresight) || !TryNormalize(look_ecef, &look)) {
    return false;
  }
  const double dot = std::max(
      -1.0, std::min(1.0, boresight.x * look.x + boresight.y * look.y + boresight.z * look.z));
  const double off_axis_deg = std::acos(dot) * 180.0 / kPi;
  const double half_beamwidth_deg = 0.5 * pattern.half_power_beamwidth_deg;
  double candidate = pattern.peak_gain_dbi;
  if (off_axis_deg <= half_beamwidth_deg) {
    const double normalized_offset = off_axis_deg / pattern.half_power_beamwidth_deg;
    candidate -= 12.0 * normalized_offset * normalized_offset;
  } else if (off_axis_deg < 90.0) {
    candidate += pattern.sidelobe_level_db;
  } else {
    candidate += pattern.backlobe_level_db;
  }
  if (!IsFinite(candidate)) {
    return false;
  }
  *gain_dbi = candidate;
  return true;
}

bool IsLinear(RfPolarization polarization) {
  return polarization == RfPolarization::kHorizontal || polarization == RfPolarization::kVertical;
}

bool IsCircular(RfPolarization polarization) {
  return polarization == RfPolarization::kRightHandCircular ||
         polarization == RfPolarization::kLeftHandCircular;
}

/** @brief 使用名义极化和天线正交隔离计算保守极化失配损耗。 */
bool TryPolarizationLoss(const RfEmission& emission, const RfReceiverSite& receiver,
                         double* loss_db) {
  if (loss_db == nullptr || !IsKnownPolarization(emission.polarization) ||
      !IsKnownPolarization(receiver.polarization)) {
    return false;
  }
  double candidate = 0.0;
  if (emission.polarization == RfPolarization::kUnpolarized ||
      receiver.polarization == RfPolarization::kUnpolarized ||
      (IsLinear(emission.polarization) && IsCircular(receiver.polarization)) ||
      (IsCircular(emission.polarization) && IsLinear(receiver.polarization))) {
    candidate = kLinearCircularLossDb;
  } else if (emission.polarization != receiver.polarization) {
    candidate = std::min(emission.antenna.cross_polarization_isolation_db,
                         receiver.antenna.cross_polarization_isolation_db);
  }
  *loss_db = candidate;
  return true;
}

bool IsValidSegment(const RfEmissionSegment& segment) {
  return IsFinite(segment.start_time_s) && IsFinite(segment.duration_s) &&
         segment.duration_s > 0.0 && IsFinite(segment.center_frequency_hz) &&
         segment.center_frequency_hz > 0.0 && IsFinite(segment.bandwidth_hz) &&
         segment.bandwidth_hz > 0.0 && IsFinite(segment.transmit_power_w) &&
         segment.transmit_power_w >= 0.0;
}

}  // namespace

bool TryRfFrequencyOverlapFraction(double emission_center_hz, double emission_bandwidth_hz,
                                   double receiver_center_hz, double receiver_bandwidth_hz,
                                   double* fraction) {
  if (fraction == nullptr || !IsFinite(emission_center_hz) || emission_center_hz <= 0.0 ||
      !IsFinite(emission_bandwidth_hz) || emission_bandwidth_hz <= 0.0 ||
      !IsFinite(receiver_center_hz) || receiver_center_hz <= 0.0 ||
      !IsFinite(receiver_bandwidth_hz) || receiver_bandwidth_hz <= 0.0) {
    return false;
  }
  const double emission_low = emission_center_hz - 0.5 * emission_bandwidth_hz;
  const double emission_high = emission_center_hz + 0.5 * emission_bandwidth_hz;
  const double receiver_low = receiver_center_hz - 0.5 * receiver_bandwidth_hz;
  const double receiver_high = receiver_center_hz + 0.5 * receiver_bandwidth_hz;
  if (!IsFinite(emission_low) || !IsFinite(emission_high) || !IsFinite(receiver_low) ||
      !IsFinite(receiver_high) || emission_low < 0.0 || receiver_low < 0.0) {
    return false;
  }
  const double overlap_hz =
      std::max(0.0, std::min(emission_high, receiver_high) - std::max(emission_low, receiver_low));
  const double candidate = overlap_hz / emission_bandwidth_hz;
  if (!IsFinite(candidate)) {
    return false;
  }
  *fraction = std::max(0.0, std::min(1.0, candidate));
  return true;
}

bool TryRfTimeOverlapFraction(double emission_start_s, double emission_duration_s,
                              double receiver_start_s, double receiver_duration_s,
                              double* fraction) {
  if (fraction == nullptr || !IsFinite(emission_start_s) || !IsFinite(emission_duration_s) ||
      emission_duration_s <= 0.0 || !IsFinite(receiver_start_s) || !IsFinite(receiver_duration_s) ||
      receiver_duration_s <= 0.0) {
    return false;
  }
  const double emission_end_s = emission_start_s + emission_duration_s;
  const double receiver_end_s = receiver_start_s + receiver_duration_s;
  if (!IsFinite(emission_end_s) || !IsFinite(receiver_end_s)) {
    return false;
  }
  const double overlap_s = std::max(
      0.0, std::min(emission_end_s, receiver_end_s) - std::max(emission_start_s, receiver_start_s));
  const double candidate = overlap_s / receiver_duration_s;
  if (!IsFinite(candidate)) {
    return false;
  }
  *fraction = std::max(0.0, std::min(1.0, candidate));
  return true;
}

bool TryValidateRfEmissionFrame(const std::vector<RfEmission>& emissions, double cycle_duration_s) {
  if (!IsFinite(cycle_duration_s) || cycle_duration_s <= 0.0) {
    return false;
  }
  std::set<std::uint64_t> emission_ids;
  for (const RfEmission& emission : emissions) {
    if (!emission_ids.insert(emission.emission_id).second || emission.segments.empty() ||
        !IsKnownWaveform(emission.waveform_kind) || !IsKnownPolarization(emission.polarization) ||
        !IsFinitePosition(emission.position_ecef_m) ||
        !IsFiniteVelocity(emission.velocity_ecef_mps) || !IsValidPattern(emission.antenna)) {
      return false;
    }
    for (const RfEmissionSegment& segment : emission.segments) {
      if (!IsValidSegment(segment) || segment.start_time_s < 0.0) {
        return false;
      }
      const double segment_end_s = segment.start_time_s + segment.duration_s;
      if (!IsFinite(segment_end_s) || segment_end_s > cycle_duration_s) {
        return false;
      }
    }
  }
  return true;
}

bool TryEvaluateRfLink(const RfEmission& emission, const RfReceiverSite& receiver,
                       const RfLinkEvaluationConfig& config, RfLinkResult* result) {
  if (result == nullptr || emission.segments.empty() || !IsKnownWaveform(emission.waveform_kind) ||
      !IsKnownPolarization(emission.polarization) || !IsKnownPolarization(receiver.polarization) ||
      !IsFinitePosition(emission.position_ecef_m) || !IsFinitePosition(receiver.position_ecef_m) ||
      !IsFiniteVelocity(emission.velocity_ecef_mps) ||
      !IsFiniteVelocity(receiver.velocity_ecef_mps) || !IsValidPattern(emission.antenna) ||
      !IsValidPattern(receiver.antenna) || !IsFinite(receiver.window_start_time_s) ||
      !IsFinite(receiver.window_duration_s) || receiver.window_duration_s <= 0.0 ||
      !IsFinite(receiver.center_frequency_hz) || receiver.center_frequency_hz <= 0.0 ||
      !IsFinite(receiver.bandwidth_hz) || receiver.bandwidth_hz <= 0.0 ||
      !IsFinite(receiver.receiver_system_loss_db) || receiver.receiver_system_loss_db < 0.0 ||
      !IsFinite(receiver.minimum_far_field_range_m) || receiver.minimum_far_field_range_m <= 0.0 ||
      !IsFinite(config.additional_propagation_loss_db) ||
      config.additional_propagation_loss_db < 0.0) {
    return false;
  }
  for (const RfEmissionSegment& segment : emission.segments) {
    if (!IsValidSegment(segment)) {
      return false;
    }
  }

  const bool is_co_site = emission.entity_id == receiver.entity_id;
  if (is_co_site && (!receiver.has_co_site_isolation || !IsFinite(receiver.co_site_isolation_db) ||
                     receiver.co_site_isolation_db < 0.0)) {
    return false;
  }

  RfEcefUnitVector emitter_to_receiver;
  emitter_to_receiver.x = receiver.position_ecef_m.x_m - emission.position_ecef_m.x_m;
  emitter_to_receiver.y = receiver.position_ecef_m.y_m - emission.position_ecef_m.y_m;
  emitter_to_receiver.z = receiver.position_ecef_m.z_m - emission.position_ecef_m.z_m;
  const double distance_squared = emitter_to_receiver.x * emitter_to_receiver.x +
                                  emitter_to_receiver.y * emitter_to_receiver.y +
                                  emitter_to_receiver.z * emitter_to_receiver.z;
  if (!IsFinite(distance_squared) || distance_squared < 0.0) {
    return false;
  }
  const double path_length_m = std::sqrt(distance_squared);
  if (!is_co_site && path_length_m < receiver.minimum_far_field_range_m) {
    return false;
  }

  RfLinkResult candidate;
  candidate.emission_id = emission.emission_id;
  candidate.emitter_entity_id = emission.entity_id;
  candidate.receiver_entity_id = receiver.entity_id;
  candidate.is_co_site = is_co_site;
  candidate.path_length_m = path_length_m;
  candidate.additional_propagation_loss_db = config.additional_propagation_loss_db;
  candidate.receiver_system_loss_db = receiver.receiver_system_loss_db;
  candidate.co_site_isolation_db = is_co_site ? receiver.co_site_isolation_db : 0.0;

  if (!is_co_site) {
    RfEcefUnitVector receiver_to_emitter;
    receiver_to_emitter.x = -emitter_to_receiver.x;
    receiver_to_emitter.y = -emitter_to_receiver.y;
    receiver_to_emitter.z = -emitter_to_receiver.z;
    if (!TryEvaluateGain(emission.antenna, emitter_to_receiver,
                         &candidate.transmit_antenna_gain_dbi) ||
        !TryEvaluateGain(receiver.antenna, receiver_to_emitter,
                         &candidate.receive_antenna_gain_dbi) ||
        !TryPolarizationLoss(emission, receiver, &candidate.polarization_mismatch_loss_db)) {
      return false;
    }
  }

  candidate.segment_results.reserve(emission.segments.size());
  for (std::size_t index = 0; index < emission.segments.size(); ++index) {
    const RfEmissionSegment& segment = emission.segments[index];
    RfSegmentLinkResult segment_result;
    segment_result.segment_index = index;
    if (!TryRfFrequencyOverlapFraction(segment.center_frequency_hz, segment.bandwidth_hz,
                                       receiver.center_frequency_hz, receiver.bandwidth_hz,
                                       &segment_result.frequency_overlap_fraction) ||
        !TryRfTimeOverlapFraction(segment.start_time_s, segment.duration_s,
                                  receiver.window_start_time_s, receiver.window_duration_s,
                                  &segment_result.time_overlap_fraction)) {
      return false;
    }

    double total_loss_db = candidate.receiver_system_loss_db;
    if (is_co_site) {
      total_loss_db += candidate.co_site_isolation_db;
    } else {
      const double wavelength_m = kSpeedOfLightMps / segment.center_frequency_hz;
      const double path_ratio = 4.0 * kPi * path_length_m / wavelength_m;
      if (!IsFinite(path_ratio) || path_ratio <= 0.0) {
        return false;
      }
      segment_result.free_space_loss_db = 20.0 * std::log10(path_ratio);
      total_loss_db += segment_result.free_space_loss_db +
                       candidate.additional_propagation_loss_db +
                       candidate.polarization_mismatch_loss_db -
                       candidate.transmit_antenna_gain_dbi - candidate.receive_antenna_gain_dbi;
    }
    const double linear_path_gain = std::pow(10.0, -total_loss_db / 10.0);
    segment_result.received_power_before_overlap_w = segment.transmit_power_w * linear_path_gain;
    segment_result.received_power_w = segment_result.received_power_before_overlap_w *
                                      segment_result.frequency_overlap_fraction *
                                      segment_result.time_overlap_fraction;
    if (!IsFinite(segment_result.received_power_before_overlap_w) ||
        !IsFinite(segment_result.received_power_w) || segment_result.received_power_w < 0.0) {
      return false;
    }
    candidate.total_received_power_w += segment_result.received_power_w;
    if (!IsFinite(candidate.total_received_power_w)) {
      return false;
    }
    candidate.segment_results.push_back(segment_result);
  }

  *result = std::move(candidate);
  return true;
}

bool TryAggregateRfReceivedPower(const std::vector<RfLinkResult>& links,
                                 double* total_received_power_w) {
  if (total_received_power_w == nullptr) {
    return false;
  }
  std::vector<std::pair<std::uint64_t, double>> ordered_powers;
  ordered_powers.reserve(links.size());
  std::uint64_t receiver_entity_id = 0;
  bool has_receiver = false;
  for (const RfLinkResult& link : links) {
    if (!IsFinite(link.total_received_power_w) || link.total_received_power_w < 0.0 ||
        (has_receiver && link.receiver_entity_id != receiver_entity_id)) {
      return false;
    }
    receiver_entity_id = link.receiver_entity_id;
    has_receiver = true;
    ordered_powers.emplace_back(link.emission_id, link.total_received_power_w);
  }
  std::sort(ordered_powers.begin(), ordered_powers.end(),
            [](const std::pair<std::uint64_t, double>& left,
               const std::pair<std::uint64_t, double>& right) { return left.first < right.first; });
  long double sum = 0.0L;
  for (std::size_t index = 0; index < ordered_powers.size(); ++index) {
    if (index > 0 && ordered_powers[index - 1].first == ordered_powers[index].first) {
      return false;
    }
    sum += static_cast<long double>(ordered_powers[index].second);
  }
  const double candidate = static_cast<double>(sum);
  if (!IsFinite(candidate)) {
    return false;
  }
  *total_received_power_w = candidate;
  return true;
}

}  // namespace electromagnetics
}  // namespace oneq
