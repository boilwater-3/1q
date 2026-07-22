#include "airborne_radar/signal/detection/ArDetectionCellResolver.h"

#include <algorithm>
#include <cmath>

namespace airborne_radar {
namespace signal {
namespace detection {
namespace {

constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kBoltzmannJPerK = 1.380649e-23;
constexpr double kPi = 3.14159265358979323846;
constexpr double kLinearFloor = 1.0e-300;

bool IsFinitePositive(double value) { return std::isfinite(value) && value > 0.0; }

bool SameIdentity(const oneq::electromagnetics::RfEmissionIdentity& left,
                  const oneq::electromagnetics::RfEmissionIdentity& right) {
  return left.platform_id == right.platform_id && left.equipment_id == right.equipment_id &&
         left.emission_id == right.emission_id;
}

}  // namespace

bool TryResolveArDetectionCell(
    const ArDetectionCellConfig& config, const ArDetectionCellTarget& target,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    double clutter_power_w, ArDetectionCellResult* result) {
  if (result == nullptr || !IsFinitePositive(config.carrier_frequency_hz) ||
      !IsFinitePositive(config.matched_filter_bandwidth_hz) ||
      !IsFinitePositive(config.pulse_width_s) || !IsFinitePositive(config.radiated_peak_power_w) ||
      !std::isfinite(config.one_way_antenna_gain_dbi) || !std::isfinite(config.receiver_loss_db) ||
      config.receiver_loss_db < 0.0 || !std::isfinite(config.receiver_noise_figure_db) ||
      config.receiver_noise_figure_db < 0.0 || !IsFinitePositive(config.reference_temperature_k) ||
      !IsFinitePositive(target.range_m) || !std::isfinite(target.closing_radial_velocity_mps) ||
      !IsFinitePositive(target.rcs_m2) ||
      !std::isfinite(target.two_way_additional_propagation_loss_db) ||
      target.two_way_additional_propagation_loss_db < 0.0 || target.effective_pulse_count == 0U ||
      !std::isfinite(clutter_power_w) || clutter_power_w < 0.0) {
    return false;
  }

  long double interference_power_w = 0.0L;
  for (const auto& link : incident_links) {
    if (!std::isfinite(link.received_power_w) || link.received_power_w < 0.0) {
      return false;
    }
    if (!SameIdentity(link.identity, own_emission_identity)) {
      interference_power_w += static_cast<long double>(link.received_power_w);
    }
  }
  const double aggregated_interference_power_w = static_cast<double>(interference_power_w);
  if (!std::isfinite(aggregated_interference_power_w)) {
    return false;
  }

  ArDetectionCellResult candidate;
  const double wavelength_m = kSpeedOfLightMps / config.carrier_frequency_hz;
  const double antenna_gain_linear = std::pow(10.0, config.one_way_antenna_gain_dbi / 10.0);
  const double total_loss_linear = std::pow(
      10.0, (config.receiver_loss_db + target.two_way_additional_propagation_loss_db) / 10.0);
  const double range_squared_m2 = target.range_m * target.range_m;
  const double range_fourth_m4 = range_squared_m2 * range_squared_m2;
  const double geometric_denominator = std::pow(4.0 * kPi, 3.0) * range_fourth_m4;
  candidate.echo_power_w = config.radiated_peak_power_w * antenna_gain_linear *
                           antenna_gain_linear * wavelength_m * wavelength_m * target.rcs_m2 /
                           (geometric_denominator * total_loss_linear);
  candidate.echo_delay_s = 2.0 * target.range_m / kSpeedOfLightMps;
  candidate.two_way_doppler_shift_hz =
      2.0 * config.carrier_frequency_hz * target.closing_radial_velocity_mps / kSpeedOfLightMps;
  candidate.pulse_compression_gain =
      std::max(1.0, config.matched_filter_bandwidth_hz * config.pulse_width_s);
  const double noise_figure_linear = std::pow(10.0, config.receiver_noise_figure_db / 10.0);
  candidate.thermal_noise_power_w = kBoltzmannJPerK * config.reference_temperature_k *
                                    config.matched_filter_bandwidth_hz * noise_figure_linear;
  candidate.interference_power_w = aggregated_interference_power_w;
  candidate.clutter_power_w = clutter_power_w;
  const double denominator = std::max(
      candidate.thermal_noise_power_w + candidate.interference_power_w + candidate.clutter_power_w,
      kLinearFloor);
  candidate.processed_single_pulse_sinr_linear =
      candidate.echo_power_w * candidate.pulse_compression_gain / denominator;
  candidate.processed_single_pulse_sinr_db =
      10.0 * std::log10(std::max(candidate.processed_single_pulse_sinr_linear, kLinearFloor));
  candidate.effective_pulse_count = target.effective_pulse_count;
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
