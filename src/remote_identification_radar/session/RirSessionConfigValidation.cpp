/**
 * @file RirSessionConfigValidation.cpp
 * @brief 远程识别雷达会话配置校验实现。
 *
 * 校验逻辑为历史 AR 会话配置校验中的识别校验段（审计基线 96de367c）
 * 的平移改写：识别策略字段路径改为 `policy.recognition.*` 域内、识别任务
 * 作用距离/驻留字段四域归位至 `mission.*` 域内；issue code 改为
 * `rir.validation.recognition_*`（RirIssueCodes.h），语义与门限值不变。
 */

#include "1q/remote_identification_radar/config/RirSessionConfigValidation.h"

#include <cmath>

#include "1q/remote_identification_radar/session/RirIssueCodes.h"
#include "common/validation/ValidationUtils.h"

namespace remote_identification_radar {
namespace config {

namespace {

bool IsFinite(float value) { return std::isfinite(value); }

void PushIssue(session::RirIssueList* issues, const char* code, const char* field,
               const std::string& message) {
  session::RirIssue issue;
  issue.severity = session::RirIssueSeverity::kError;
  issue.phase = session::RirIssuePhase::kInputValidation;
  issue.code = code;
  issue.field = field;
  issue.message = message;
  issues->push_back(issue);
}

void ValidateRirEnvironmentConfig(const RirEnvironmentConfig& environment,
                                  const char* field_prefix, session::RirIssueList* issues) {
  const std::string prefix = field_prefix == nullptr ? "" : std::string(field_prefix) + ".";
  if (!IsFinite(environment.weather_attenuation_db) || environment.weather_attenuation_db < 0.0f) {
    PushIssue(issues, session::codes::kInvalidEnvironmentSnapshot,
              (prefix + "weather_attenuation_db").c_str(),
              "Weather attenuation must be finite and non-negative.");
  }
  const RirVegetationCoverProfile cover =
      environment.vegetation_scatter_physics.cover_profile;
  if (cover < RirVegetationCoverProfile::kDisabled ||
      cover > RirVegetationCoverProfile::kTropicalDense) {
    PushIssue(issues, session::codes::kInvalidEnvironmentSnapshot,
              (prefix + "vegetation_scatter_physics.cover_profile").c_str(),
              "Vegetation cover profile must be a valid enum value.");
  }
  const RirAtmosphericPhysicsConfig& atmospheric = environment.atmospheric_physics;
  if (!IsFinite(atmospheric.pressure_hpa) || atmospheric.pressure_hpa <= 0.0f ||
      !IsFinite(atmospheric.temperature_k) || atmospheric.temperature_k <= 0.0f ||
      !IsFinite(atmospheric.relative_humidity) || atmospheric.relative_humidity < 0.0f ||
      atmospheric.relative_humidity > 1.0f) {
    PushIssue(issues, session::codes::kInvalidEnvironmentSnapshot,
              (prefix + "atmospheric_physics").c_str(),
              "Atmospheric observation must be finite with pressure/temperature > 0 and "
              "relative humidity in [0, 1].");
  }
}

void ValidateRirHardwareConfig(const RirHardwareConfig& hardware, session::RirIssueList* issues) {
  const hardware::RirTransmitterConfig& transmitter = hardware.transmitter;
  const hardware::RirReceiverConfig& receiver = hardware.receiver;
  const hardware::RirAntennaConfig& antenna = hardware.antenna;
  const hardware::RirRcsPhysicsConfig& rcs_physics = hardware.rcs_physics;
  const float transmitter_frequency_hz = transmitter.frequency_hz;

  if (!oneq::common::validation::IsFinite(transmitter_frequency_hz) ||
      transmitter_frequency_hz <= 0.0f) {
    PushIssue(issues, session::codes::kTransmitterFrequencyInvalid,
              "hardware.transmitter.frequency_hz",
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
    PushIssue(issues, session::codes::kFrequencyPlanInvalid,
              "hardware.transmitter.frequency_plan_hz",
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
    PushIssue(issues, session::codes::kTransmitterOperatingEnvelopeInvalid, "hardware.transmitter",
              "Transmitter power, duty cycle and pulse energy must stay inside hardware limits.");
  }
  if (transmitter.equipment_id == 0U || receiver.equipment_id == 0U ||
      transmitter.equipment_id == receiver.equipment_id) {
    PushIssue(issues, session::codes::kEquipmentIdentityInvalid, "hardware.*.equipment_id",
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
    PushIssue(issues, session::codes::kReceiverRfHardwareInvalid, "hardware.receiver",
              "Receiver RF isolation, far-field range and linear input limit must be valid.");
  }
  for (const auto& path : receiver.co_site_paths) {
    if (path.transmitter_equipment_id == 0U ||
        path.receiver_equipment_id != receiver.equipment_id ||
        path.transmitter_equipment_id == path.receiver_equipment_id ||
        !oneq::common::validation::IsFinite(path.isolation_db) || path.isolation_db < 0.0) {
      PushIssue(issues, session::codes::kReceiverRfHardwareInvalid,
                "hardware.receiver.co_site_paths",
                "Each co-site path must be a valid directed path into the receiver equipment.");
      break;
    }
  }

  const auto axis_geometry_valid = [transmitter_frequency_hz](float nominal_beamwidth_deg,
                                                              float aperture_m) {
    if (!oneq::common::validation::IsFinite(nominal_beamwidth_deg) ||
        !oneq::common::validation::IsFinite(aperture_m) || nominal_beamwidth_deg < 0.0f ||
        aperture_m < 0.0f) {
      return false;
    }
    if (nominal_beamwidth_deg > 0.0f) {
      return true;
    }
    return aperture_m > 0.0f && oneq::common::validation::IsFinite(transmitter_frequency_hz) &&
           transmitter_frequency_hz > 0.0f;
  };

  if (!axis_geometry_valid(antenna.nominal_az_beamwidth_deg, antenna.antenna_length_m)) {
    PushIssue(issues, session::codes::kAntennaAzGeometryInvalid,
              "hardware.antenna.nominal_az_beamwidth_deg / antenna_length_m",
              "Azimuth beamwidth requires a positive nominal value or a valid physical aperture.");
  }
  if (!axis_geometry_valid(antenna.nominal_el_beamwidth_deg, antenna.antenna_width_m)) {
    PushIssue(issues, session::codes::kAntennaElGeometryInvalid,
              "hardware.antenna.nominal_el_beamwidth_deg / antenna_width_m",
              "Elevation beamwidth requires a positive nominal value or a valid physical aperture.");
  }

  if (!oneq::common::validation::IsFinite(rcs_physics.physics_mix_ratio) ||
      rcs_physics.physics_mix_ratio < 0.0f || rcs_physics.physics_mix_ratio > 1.0f ||
      !oneq::common::validation::IsFinite(rcs_physics.cylinder_weight) ||
      rcs_physics.cylinder_weight < 0.0f || rcs_physics.cylinder_weight > 1.0f ||
      !oneq::common::validation::IsFinite(rcs_physics.min_equivalent_radius_m) ||
      !oneq::common::validation::IsFinite(rcs_physics.max_equivalent_radius_m) ||
      rcs_physics.min_equivalent_radius_m <= 0.0f ||
      rcs_physics.max_equivalent_radius_m < rcs_physics.min_equivalent_radius_m ||
      !oneq::common::validation::IsFinite(rcs_physics.min_rcs_m2) ||
      !oneq::common::validation::IsFinite(rcs_physics.max_rcs_m2) ||
      rcs_physics.min_rcs_m2 < 0.0f || rcs_physics.max_rcs_m2 < rcs_physics.min_rcs_m2 ||
      !oneq::common::validation::IsFinite(rcs_physics.bistatic_psi_offset_deg)) {
    PushIssue(issues, session::codes::kRcsPhysicsInvalid, "hardware.rcs_physics",
              "RCS physics parameters must be finite with ordered radius and RCS bounds.");
  }
}

}  // namespace

session::RirIssueList ValidateRirSessionConfig(const RirSessionConfig& config) {
  session::RirIssueList issues;
  if (config.sensor_platform_id == 0U) {
    PushIssue(&issues, session::codes::kSensorPlatformIdInvalid, "sensor_platform_id",
              "Sensor platform identifier must be non-zero.");
  }
  const RirMissionConfig& mission = config.mission;
  const RirRecognitionPolicy& recognition = config.policy.recognition;
  const RirRecognitionFeatureWeights& weights = recognition.feature_weights;

  // 权重：有限、[0, 1]、四者之和为 1.0。
  const float weight_sum = weights.rcs_weight + weights.motion_weight +
                           weights.polarization_weight + weights.range_profile_weight;
  if (!IsFinite(weights.rcs_weight) || !IsFinite(weights.motion_weight) ||
      !IsFinite(weights.polarization_weight) || !IsFinite(weights.range_profile_weight) ||
      weights.rcs_weight < 0.0f || weights.rcs_weight > 1.0f || weights.motion_weight < 0.0f ||
      weights.motion_weight > 1.0f || weights.polarization_weight < 0.0f ||
      weights.polarization_weight > 1.0f || weights.range_profile_weight < 0.0f ||
      weights.range_profile_weight > 1.0f || std::fabs(weight_sum - 1.0f) > 1.0e-5f) {
    PushIssue(&issues, session::codes::kRecognitionWeightsInvalid,
              "policy.recognition.feature_weights",
              "Recognition feature weights must be finite values in [0, 1] summing to 1.0.");
  }

  // 数据库路径：启用识别时必须非空。
  if (recognition.enabled && recognition.database_path.empty()) {
    PushIssue(&issues, session::codes::kRecognitionDatabasePathMissing,
              "policy.recognition.database_path",
              "Recognition database path must be non-empty when recognition is enabled.");
  }

  // 判定门限：[0, 1]。
  if (!IsFinite(recognition.acceptance_score) || recognition.acceptance_score < 0.0f ||
      recognition.acceptance_score > 1.0f || !IsFinite(recognition.minimum_margin) ||
      recognition.minimum_margin < 0.0f || recognition.minimum_margin > 1.0f) {
    PushIssue(&issues, session::codes::kRecognitionThresholdInvalid,
              "policy.recognition.acceptance_score / minimum_margin",
              "Recognition acceptance score and minimum margin must be finite values in [0, 1].");
  }

  // 积累计数：至少为 1。
  if (recognition.min_confirmed_hits == 0U || recognition.min_observation_count == 0U) {
    PushIssue(&issues, session::codes::kRecognitionAccumulationInvalid,
              "policy.recognition.min_confirmed_hits / min_observation_count",
              "Recognition accumulation counts must be at least 1.");
  }

  // 识别策略时间范围：保持时间非负；累积窗口有限且为正。
  if (!IsFinite(recognition.result_hold_sec) || recognition.result_hold_sec < 0.0f ||
      !IsFinite(recognition.accumulation_window_sec) ||
      recognition.accumulation_window_sec <= 0.0f) {
    PushIssue(&issues, session::codes::kRecognitionTimeRangeInvalid,
              "policy.recognition.result_hold_sec / accumulation_window_sec",
              "Recognition hold time must be non-negative and accumulation window must be "
              "finite and positive.");
  }

  // 任务域识别作用范围/驻留：最大距离与驻留时间有限且为正（四域归位后）。
  if (!IsFinite(mission.max_range_m) || mission.max_range_m <= 0.0f ||
      !IsFinite(mission.recognition_dwell_sec) || mission.recognition_dwell_sec <= 0.0f) {
    PushIssue(&issues, session::codes::kRecognitionTimeRangeInvalid,
              "mission.max_range_m / recognition_dwell_sec",
              "Recognition max range and dwell must be finite and positive.");
  }

  // 扫描策略（库内驻留调度器）：步长系数有限且为正。
  {
    const RirScanConfig& scan = mission.scan;
    if (!IsFinite(scan.step_scale) || scan.step_scale <= 0.0f) {
      PushIssue(&issues, session::codes::kScanStrategyInvalid, "mission.scan",
                "Scan step scale must be finite and positive.");
    }
  }

  // 任务扫描子窗（用户指定作战搜索扇区）：az 相对域 [-180,180]、el 绝对域 [-90,90]，
  // 且逐轴有序（min ≤ max）。缺省无界满足约束；实际搜索扇区为其与体积的交集。
  {
    const RirAzimuthElevationLimitsDeg& window = mission.scan_window_deg;
    if (!IsFinite(window.az_min_deg) || !IsFinite(window.az_max_deg) ||
        !IsFinite(window.el_min_deg) || !IsFinite(window.el_max_deg) ||
        window.az_min_deg > window.az_max_deg || window.el_min_deg > window.el_max_deg ||
        window.az_min_deg < -180.0f || window.az_max_deg > 180.0f ||
        window.el_min_deg < -90.0f || window.el_max_deg > 90.0f) {
      PushIssue(&issues, session::codes::kScanStrategyInvalid, "mission.scan_window_deg",
                "Mission scan window limits must be finite, ordered, az in [-180,180] relative "
                "domain and el in [-90,90].");
    }
  }

  // 可扫描体积（orientation 第五域）：az 相对域 [-180,180]、el 绝对域 [-90,90]。
  {
    const RirAzimuthElevationLimitsDeg& volume = config.orientation.steerable_volume_deg;
    if (!IsFinite(volume.az_min_deg) || !IsFinite(volume.az_max_deg) ||
        !IsFinite(volume.el_min_deg) || !IsFinite(volume.el_max_deg) ||
        volume.az_min_deg > volume.az_max_deg || volume.el_min_deg > volume.el_max_deg ||
        volume.az_min_deg < -180.0f || volume.az_max_deg > 180.0f ||
        volume.el_min_deg < -90.0f || volume.el_max_deg > 90.0f) {
      PushIssue(&issues, session::codes::kSteerableVolumeInvalid,
                "orientation.steerable_volume_deg",
                "Steerable volume limits must be finite, ordered, az in [-180,180] relative "
                "domain and el in [-90,90].");
    }
  }

  // 转台朝向（mission.scan_center_deg）：有限且在合法域。
  {
    const RirAzimuthElevationDeg& center = mission.scan_center_deg;
    if (!IsFinite(center.az_deg) || !IsFinite(center.el_deg) || center.az_deg < -180.0f ||
        center.az_deg > 180.0f || center.el_deg < -90.0f || center.el_deg > 90.0f) {
      PushIssue(&issues, session::codes::kScanCenterInvalid, "mission.scan_center_deg",
                "Scan center must be finite with az in [-180,180] and el in [-90,90].");
    }
  }

  // 自持检测策略（阶段 2-S）：Pfa/SNR 门限有限且范围合理，脉冲数/种子为正。
  {
    const RirDetectionPolicyConfig& detection = config.policy.detection;
    if (!IsFinite(detection.cfar_pfa) || detection.cfar_pfa <= 0.0f || detection.cfar_pfa > 1.0f ||
        !IsFinite(detection.min_snr_db) || !IsFinite(detection.min_detection_margin_db) ||
        detection.pulse_count <= 0 || detection.random_seed == 0U) {
      PushIssue(&issues, session::codes::kDetectionPolicyInvalid, "policy.detection",
                "Detection policy Pfa/SNR gates must be finite, pulse count and seed positive.");
    }
  }

  // 关联策略：波门 sigma 有限且 > 0。
  {
    const RirAssociationPolicyConfig& association = config.policy.association;
    if (!IsFinite(association.distance_gate_sigma) || association.distance_gate_sigma <= 0.0f) {
      PushIssue(&issues, session::codes::kAssociationPolicyInvalid, "policy.association",
                "Association distance gate sigma must be finite and positive.");
    }
  }

  // 跟踪策略：KF 噪声参数有限且 > 0。
  {
    const RirTrackingPolicyConfig& tracking = config.policy.tracking;
    if (!IsFinite(tracking.kalman_noise_diff_coeff) || tracking.kalman_noise_diff_coeff <= 0.0f ||
        !IsFinite(tracking.kalman_measurement_noise_std) ||
        tracking.kalman_measurement_noise_std <= 0.0f) {
      PushIssue(&issues, session::codes::kTrackingPolicyInvalid, "policy.tracking",
                "Kalman process/measurement noise parameters must be finite and positive.");
    }
  }

  // 生命周期策略：confirm_hits/max_lost_cycles 至少为 1。
  {
    const RirLifecyclePolicyConfig& lifecycle = config.policy.lifecycle;
    if (lifecycle.confirm_hits == 0U || lifecycle.max_lost_cycles == 0U) {
      PushIssue(&issues, session::codes::kLifecyclePolicyInvalid, "policy.lifecycle",
                "Lifecycle confirm hits and max lost cycles must be at least 1.");
    }
  }

  // 信号处理增益偏置（阶段 2-M M3）：有限且 [0, 40] dB。
  {
    const RirHardwareConfig& hardware = config.hardware;
    const hardware::RirSignalProcessingConfig& gains = hardware.signal_processing;
    if (!IsFinite(gains.target_processing_gain_db) || !IsFinite(gains.noise_processing_gain_db) ||
        !IsFinite(gains.clutter_suppression_gain_db) ||
        !IsFinite(gains.jamming_suppression_gain_db) || gains.target_processing_gain_db < 0.0f ||
        gains.target_processing_gain_db > 40.0f || gains.noise_processing_gain_db < 0.0f ||
        gains.noise_processing_gain_db > 40.0f || gains.clutter_suppression_gain_db < 0.0f ||
        gains.clutter_suppression_gain_db > 40.0f || gains.jamming_suppression_gain_db < 0.0f ||
        gains.jamming_suppression_gain_db > 40.0f) {
      PushIssue(&issues, session::codes::kSignalProcessingGainsInvalid,
                "hardware.signal_processing",
                "Signal processing gain offsets must be finite values in [0, 40] dB.");
    }
  }

  ValidateRirHardwareConfig(config.hardware, &issues);

  ValidateRirEnvironmentConfig(config.environment, "environment", &issues);

  return issues;
}

}  // namespace config
}  // namespace remote_identification_radar
