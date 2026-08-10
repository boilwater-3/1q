// @file ArSessionConfigBuilder.cpp
// @brief Implementation of ArSessionConfigBuilder (thin wrapper).

#include "1q/airborne_radar/config/ArSessionConfigBuilder.h"

#include "1q/airborne_radar/config/ArSessionConfigValidation.h"
#include "1q/airborne_radar/session/ArIssueCodes.h"
#include "common/validation/ValidationUtils.h"

namespace airborne_radar {
namespace config {

ArSessionConfigBuilder::ArSessionConfigBuilder() = default;

ArSessionConfigBuilder::ArSessionConfigBuilder(const config::ArSessionConfig& config)
    : config_(config) {}

config::ArSessionConfig ArSessionConfigBuilder::Build() const { return config_; }

session::ArIssueList ValidateArSessionConfig(const config::ArSessionConfig& config) noexcept {
  session::ArIssueList issues;
  const auto push = [&issues](const char* code, const char* field, const char* msg) {
    session::ArIssue issue;
    issue.severity = session::ArIssueSeverity::kError;
    issue.phase = session::ArIssuePhase::kInputValidation;
    issue.code = code;
    issue.field = field;
    issue.message = msg;
    issues.push_back(issue);
  };
  const config::ArOrientationConfig& orientation = config.mission.orientation;
  const config::detection::AntennaConfig& antenna = config.hardware.antenna;
  const config::detection::ReceiverConfig& receiver = config.hardware.receiver;
  const config::detection::TransmitterConfig& transmitter = config.hardware.transmitter;
  const float transmitter_frequency_hz = transmitter.frequency_hz;

  if (!oneq::common::validation::IsFinite(transmitter_frequency_hz) ||
      transmitter_frequency_hz <= 0.0f) {
    push(session::codes::kTransmitterFrequencyInvalid, "hardware.transmitter.frequency_hz",
         "Transmitter frequency must be finite and positive.");
  }
  bool frequency_plan_valid = !transmitter.frequency_plan_hz.empty();
  bool contains_initial_frequency = false;
  for (double frequency_hz : transmitter.frequency_plan_hz) {
    frequency_plan_valid = frequency_plan_valid &&
                           oneq::common::validation::IsFinite(frequency_hz) && frequency_hz > 0.0;
    contains_initial_frequency =
        contains_initial_frequency || frequency_hz == static_cast<double>(transmitter_frequency_hz);
  }
  if (!frequency_plan_valid || !contains_initial_frequency) {
    push(session::codes::kFrequencyPlanInvalid, "hardware.transmitter.frequency_plan_hz",
         "Frequency plan must contain finite positive values and the initial carrier.");
  }
  const double duty_cycle =
      static_cast<double>(transmitter.pulse_width_s) * static_cast<double>(transmitter.prf_hz);
  const double pulse_energy_j = static_cast<double>(transmitter.peak_power_w) *
                                static_cast<double>(transmitter.pulse_width_s);
  if (!oneq::common::validation::IsFinite(transmitter.peak_power_w) ||
      !oneq::common::validation::IsFinite(transmitter.maximum_peak_power_w) ||
      !oneq::common::validation::IsFinite(transmitter.maximum_duty_cycle) ||
      !oneq::common::validation::IsFinite(transmitter.maximum_pulse_energy_j) ||
      transmitter.peak_power_w <= 0.0f || transmitter.maximum_peak_power_w <= 0.0f ||
      transmitter.peak_power_w > transmitter.maximum_peak_power_w ||
      transmitter.maximum_duty_cycle <= 0.0f || transmitter.maximum_duty_cycle > 1.0f ||
      duty_cycle <= 0.0 || duty_cycle > transmitter.maximum_duty_cycle ||
      transmitter.maximum_pulse_energy_j <= 0.0f ||
      pulse_energy_j > transmitter.maximum_pulse_energy_j) {
    push(session::codes::kTransmitterOperatingEnvelopeInvalid, "hardware.transmitter",
         "Transmitter power, duty cycle and pulse energy must stay inside hardware limits.");
  }
  if (transmitter.equipment_id == 0U || receiver.equipment_id == 0U ||
      transmitter.equipment_id == receiver.equipment_id) {
    push(session::codes::kEquipmentIdentityInvalid, "hardware.*.equipment_id",
         "Transmitter and receiver equipment identifiers must be non-zero and distinct.");
  }
  if (!oneq::common::validation::IsFinite(receiver.cross_polarization_isolation_db) ||
      receiver.cross_polarization_isolation_db < 0.0f ||
      !oneq::common::validation::IsFinite(receiver.minimum_far_field_range_m) ||
      receiver.minimum_far_field_range_m <= 0.0f ||
      (receiver.has_co_site_isolation &&
       (!oneq::common::validation::IsFinite(receiver.co_site_isolation_db) ||
        receiver.co_site_isolation_db < 0.0f)) ||
      !oneq::common::validation::IsFinite(receiver.maximum_linear_input_power_w) ||
      receiver.maximum_linear_input_power_w <= 0.0f ||
      !oneq::common::validation::IsFinite(receiver.preselector_bandwidth_hz) ||
      receiver.preselector_bandwidth_hz <= 0.0f ||
      !oneq::common::validation::IsFinite(receiver.interference_observation_jn_gate_db)) {
    push(session::codes::kReceiverRfHardwareInvalid, "hardware.receiver",
         "Receiver RF isolation, far-field range and linear input limit must be valid.");
  }
  for (const auto& path : receiver.co_site_paths) {
    if (path.transmitter_equipment_id == 0U ||
        path.receiver_equipment_id != receiver.equipment_id ||
        path.transmitter_equipment_id == path.receiver_equipment_id ||
        !oneq::common::validation::IsFinite(path.isolation_db) || path.isolation_db < 0.0) {
      push(session::codes::kReceiverRfHardwareInvalid, "hardware.receiver.co_site_paths",
           "Each co-site path must be a valid directed path into the receiver equipment.");
      break;
    }
  }
  const auto axis_geometry_valid = [transmitter_frequency_hz](float nominal_beamwidth_deg,
                                                              float aperture_m,
                                                              bool commanded_override_enabled) {
    if (!oneq::common::validation::IsFinite(nominal_beamwidth_deg) ||
        !oneq::common::validation::IsFinite(aperture_m) || nominal_beamwidth_deg < 0.0f ||
        aperture_m < 0.0f) {
      return false;
    }
    if (commanded_override_enabled || nominal_beamwidth_deg > 0.0f) {
      return true;
    }
    return aperture_m > 0.0f && oneq::common::validation::IsFinite(transmitter_frequency_hz) &&
           transmitter_frequency_hz > 0.0f;
  };

  if (orientation.commanded_beamwidth_enabled) {
    if (!oneq::common::validation::IsFinite(
            orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg) ||
        orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg <= 0.0f) {
      push(session::codes::kCommandedBeamwidthAzNotPositive,
           "mission.orientation.commanded_beamwidth_deg.commanded_az_beamwidth_deg",
           "Commanded azimuth beamwidth must be finite and positive when enabled.");
    }
    if (!oneq::common::validation::IsFinite(
            orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg) ||
        orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg <= 0.0f) {
      push(session::codes::kCommandedBeamwidthElNotPositive,
           "mission.orientation.commanded_beamwidth_deg.commanded_el_beamwidth_deg",
           "Commanded elevation beamwidth must be finite and positive when enabled.");
    }
  }

  if (!axis_geometry_valid(antenna.nominal_az_beamwidth_deg, antenna.antenna_length_m,
                           orientation.commanded_beamwidth_enabled)) {
    push(session::codes::kAntennaAzGeometryInvalid,
         "hardware.antenna.nominal_az_beamwidth_deg / antenna_length_m",
         "Azimuth beamwidth requires a positive nominal value or a valid physical aperture.");
  }
  if (!axis_geometry_valid(antenna.nominal_el_beamwidth_deg, antenna.antenna_width_m,
                           orientation.commanded_beamwidth_enabled)) {
    push(session::codes::kAntennaElGeometryInvalid,
         "hardware.antenna.nominal_el_beamwidth_deg / antenna_width_m",
         "Elevation beamwidth requires a positive nominal value or a valid physical aperture.");
  }

  if (orientation.mechanical_scan_limits_deg.az_min_deg >
      orientation.mechanical_scan_limits_deg.az_max_deg) {
    push(session::codes::kMechanicalScanLimitsSwappedAz,
         "mission.orientation.mechanical_scan_limits_deg",
         "Mechanical azimuth scan min exceeds max.");
  }
  if (orientation.mechanical_scan_limits_deg.el_min_deg >
      orientation.mechanical_scan_limits_deg.el_max_deg) {
    push(session::codes::kMechanicalScanLimitsSwappedEl,
         "mission.orientation.mechanical_scan_limits_deg",
         "Mechanical elevation scan min exceeds max.");
  }

  if (orientation.electronic_scan_limits_deg.az_min_deg >
      orientation.electronic_scan_limits_deg.az_max_deg) {
    push(session::codes::kElectronicScanLimitsSwappedAz,
         "mission.orientation.electronic_scan_limits_deg",
         "Electronic azimuth scan min exceeds max.");
  }
  if (orientation.electronic_scan_limits_deg.el_min_deg >
      orientation.electronic_scan_limits_deg.el_max_deg) {
    push(session::codes::kElectronicScanLimitsSwappedEl,
         "mission.orientation.electronic_scan_limits_deg",
         "Electronic elevation scan min exceeds max.");
  }

  const config::ArRecognitionConfig& recognition = config.policy.recognition;
  const config::ArRecognitionFeatureWeights& weights = recognition.feature_weights;
  const bool weights_finite =
      oneq::common::validation::IsFinite(weights.rcs_weight) &&
      oneq::common::validation::IsFinite(weights.motion_weight) &&
      oneq::common::validation::IsFinite(weights.polarization_weight) &&
      oneq::common::validation::IsFinite(weights.range_profile_weight);
  const float weight_sum = weights.rcs_weight + weights.motion_weight +
                           weights.polarization_weight + weights.range_profile_weight;
  const bool weights_in_range =
      weights.rcs_weight >= 0.0f && weights.rcs_weight <= 1.0f &&
      weights.motion_weight >= 0.0f && weights.motion_weight <= 1.0f &&
      weights.polarization_weight >= 0.0f && weights.polarization_weight <= 1.0f &&
      weights.range_profile_weight >= 0.0f && weights.range_profile_weight <= 1.0f;
  if (!weights_finite || !weights_in_range || weight_sum < 0.999f || weight_sum > 1.001f) {
    push(session::codes::kRecognitionWeightsInvalid,
         "policy.recognition.feature_weights",
         "Recognition feature weights must be finite values in [0, 1] summing to 1.0.");
  }
  if (recognition.enabled && recognition.database_path.empty()) {
    push(session::codes::kRecognitionDatabasePathMissing,
         "policy.recognition.database_path",
         "Recognition database path must be non-empty when recognition is enabled.");
  }
  if (!oneq::common::validation::IsFinite(recognition.acceptance_score) ||
      recognition.acceptance_score < 0.0f || recognition.acceptance_score > 1.0f ||
      !oneq::common::validation::IsFinite(recognition.minimum_margin) ||
      recognition.minimum_margin < 0.0f || recognition.minimum_margin > 1.0f) {
    push(session::codes::kRecognitionThresholdInvalid,
         "policy.recognition.acceptance_score / minimum_margin",
         "Recognition acceptance score and minimum margin must be finite values in [0, 1].");
  }
  if (recognition.min_confirmed_hits == 0U || recognition.min_observation_count == 0U) {
    push(session::codes::kRecognitionAccumulationInvalid,
         "policy.recognition.min_confirmed_hits / min_observation_count",
         "Recognition accumulation counts must be at least 1.");
  }
  if (!oneq::common::validation::IsFinite(recognition.result_hold_sec) ||
      recognition.result_hold_sec < 0.0f ||
      !oneq::common::validation::IsFinite(recognition.max_range_m) ||
      recognition.max_range_m <= 0.0f ||
      !oneq::common::validation::IsFinite(recognition.recognition_dwell_sec) ||
      recognition.recognition_dwell_sec <= 0.0f ||
      !oneq::common::validation::IsFinite(recognition.accumulation_window_sec) ||
      recognition.accumulation_window_sec <= 0.0f) {
    push(session::codes::kRecognitionTimeRangeInvalid,
         "policy.recognition.result_hold_sec / max_range_m / recognition_dwell_sec / "
         "accumulation_window_sec",
         "Recognition hold time must be non-negative; max range, dwell and accumulation window "
         "must be finite and positive.");
  }

  return issues;
}

}  // namespace config
}  // namespace airborne_radar
