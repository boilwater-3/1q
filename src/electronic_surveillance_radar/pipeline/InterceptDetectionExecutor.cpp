#include "electronic_surveillance_radar/pipeline/InterceptDetectionExecutor.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>

#include "1q/electromagnetics/RfLinkBudget.h"
#include "common/geometry/GeometryTransform.h"
#include "common/logging/ProjectLog.h"
#include "common/numerics/Constants.h"
#include "common/numerics/NumericGuard.h"
#include "common/timing/TimingRegimeModel.h"
#include "common/validation/ValidationUtils.h"
#include "electronic_surveillance_radar/environment/EsrSharedUtils.h"
#include "electronic_surveillance_radar/pipeline/AngleErrorModel.h"
#include "electronic_surveillance_radar/pipeline/BoundarySearchSolver.h"
#include "electronic_surveillance_radar/pipeline/InterceptComponentFactory.h"
#include "electronic_surveillance_radar/pipeline/InterceptGate.h"
#include "electronic_surveillance_radar/pipeline/JammingAggregator.h"
#include "electronic_surveillance_radar/pipeline/ScanPatternGenerator.h"

namespace electronic_surveillance_radar {
namespace pipeline {

using oneq::common::numerics::kNumericFloor;

namespace {

double ResolveSpectrumOccupancyNoiseScale(float spectrum_occupancy_ratio) {
  return 1.0 + 9.0 * static_cast<double>(utils::Clamp01(spectrum_occupancy_ratio));
}

/**
 * @brief 判断发射源波束状态是否仍为历史默认值。
 * @param[in] beam_state 发射源波束状态。
 * @return 历史默认值返回 `true`。
 */
bool IsLegacyDefaultBeamState(const session::EsrEmitterBeamState& beam_state) {
  return !beam_state.beam_state_valid;
}

/**
 * @brief 把线性功率比转换为 dB。
 * @param[in] ratio 线性功率比。
 * @return dB 表示值。
 */
float ToDb(double ratio) {
  return static_cast<float>(10.0 * std::log10(std::max(ratio, kNumericFloor)));
}

/**
 * @brief 把 ESR 积累模式映射为共享时序体制积累模式。
 * @param[in] mode ESR 统计检测积累模式。
 * @return 共享时序体制积累模式。
 */
oneq::common::timing::IntegrationMode ToTimingIntegrationMode(
    extension::InterceptIntegrationMode mode) {
  if (mode == extension::InterceptIntegrationMode::kCoherent) {
    return oneq::common::timing::IntegrationMode::kCoherent;
  }
  return oneq::common::timing::IntegrationMode::kNonCoherent;
}

/**
 * @brief 把 ESR 统计检测配置映射为共享时序体制参数。
 * @param[in] config ESR 统计检测配置。
 * @return 共享时序体制参数。
 */
oneq::common::timing::StatisticalDetectionParams ToTimingDetectionParams(
    const extension::InterceptStatisticalDetectionConfig& config) {
  oneq::common::timing::StatisticalDetectionParams params;
  params.pfa = config.pfa;
  params.min_snr_db = config.min_snr_db;
  params.pulse_count = config.pulse_count;
  params.integration_mode = ToTimingIntegrationMode(config.integration_mode);
  params.threshold_scale = config.threshold_scale;
  params.enable_statistical_detection = config.enable_statistical_detection;
  return params;
}

/**
 * @brief 计算平台与辐射源之间的欧氏距离。
 * @param[in] platform_pose 平台状态。
 * @param[in] emitter_state 辐射源状态。
 * @return 两者距离（单位：m）。
 */
float ComputeRangeM(const oneq::foundation::PoseState& platform_pose,
                    const session::EsrSceneEmitter& emitter_state) {
  const float dx = emitter_state.pose.position_m.x - platform_pose.position_m.x;
  const float dy = emitter_state.pose.position_m.y - platform_pose.position_m.y;
  const float dz = emitter_state.pose.position_m.z - platform_pose.position_m.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief 把 ESR 三维向量映射为共享几何三维向量。
 * @param[in] vector ESR 三维向量。
 * @return 共享几何三维向量。
 */
oneq::common::geometry::Vector3f ToGeometryVector(const session::EsrVector3f& vector) {
  oneq::common::geometry::Vector3f result;
  result.x = vector.x;
  result.y = vector.y;
  result.z = vector.z;
  return result;
}

/**
 * @brief 把 ESR 欧拉角映射为共享几何欧拉角。
 * @param[in] angle ESR 欧拉角。
 * @return 共享几何欧拉角。
 */
oneq::common::geometry::EulerAnglesDeg ToGeometryEuler(const session::EsrEulerAngleDeg& angle) {
  oneq::common::geometry::EulerAnglesDeg result;
  result.yaw_deg = angle.yaw_deg;
  result.pitch_deg = angle.pitch_deg;
  result.roll_deg = angle.roll_deg;
  return result;
}

/**
 * @brief 计算辐射源相对平台在接收机参考系下的视线角。
 * @param[in] platform_pose 平台状态。
 * @param[in] emitter_state 辐射源状态。
 * @return 接收机参考系下的方位/俯仰角（单位：deg）。
 */
oneq::common::geometry::AzimuthElevationDeg ComputeEmitterLookAngles(
    const oneq::foundation::PoseState& platform_pose,
    const session::EsrSceneEmitter& emitter_state) {
  return oneq::common::geometry::ComputeRelativeLineOfSightAzEl(
      ToGeometryVector(platform_pose.position_m), ToGeometryEuler(platform_pose.attitude_deg),
      ToGeometryVector(emitter_state.pose.position_m));
}

/**
 * @brief 把平台机体参考系视线角转换为天线参考系视线角。
 * @param[in] look_angles 机体参考系方位/俯仰角。
 * @param[in] runtime_config 会话运行态配置。
 * @return 天线参考系方位/俯仰角。
 */
oneq::common::geometry::AzimuthElevationDeg ApplyAntennaMountOffset(
    const oneq::common::geometry::AzimuthElevationDeg& look_angles,
    const extension::InterceptRuntimeConfig& runtime_config) {
  oneq::common::geometry::AzimuthElevationDeg adjusted = look_angles;
  adjusted.az_deg = oneq::common::geometry::ComputeAzimuthDifferenceDeg(
      look_angles.az_deg, runtime_config.antenna_mount_az_deg);
  adjusted.el_deg = look_angles.el_deg - runtime_config.antenna_mount_el_deg;
  return adjusted;
}

/**
 * @brief 计算平台是否落入发射源当前辐射波束，并返回重叠比例。
 * @param[in] platform_pose 平台状态。
 * @param[in] emitter_state 发射源状态。
 * @return 发射源波束覆盖比例，范围 [0, 1]。
 */
float ComputeEmitterBeamOverlapRatio(const oneq::foundation::PoseState& platform_pose,
                                     const session::EsrSceneEmitter& emitter_state) {
  if (IsLegacyDefaultBeamState(emitter_state.beam_state)) {
    return 1.0f;
  }

  const oneq::common::geometry::AzimuthElevationDeg emitter_to_platform =
      oneq::common::geometry::ComputeRelativeLineOfSightAzEl(
          ToGeometryVector(emitter_state.pose.position_m),
          ToGeometryEuler(emitter_state.pose.attitude_deg),
          ToGeometryVector(platform_pose.position_m));
  const double az_diff =
      std::fabs(static_cast<double>(oneq::common::geometry::ComputeAzimuthDifferenceDeg(
          emitter_to_platform.az_deg, static_cast<float>(emitter_state.beam_state.center_az_deg))));
  const double el_diff = std::fabs(static_cast<double>(emitter_to_platform.el_deg) -
                                   emitter_state.beam_state.center_el_deg);
  const double half_az_width = std::max(1.0e-6, 0.5 * emitter_state.beam_state.az_beamwidth_deg);
  const double half_el_width = std::max(1.0e-6, 0.5 * emitter_state.beam_state.el_beamwidth_deg);
  const double normalized_az = az_diff / half_az_width;
  const double normalized_el = el_diff / half_el_width;
  const double normalized_distance =
      std::sqrt(normalized_az * normalized_az + normalized_el * normalized_el);
  if (normalized_distance >= 1.0) {
    return 0.0f;
  }
  return static_cast<float>(1.0 - normalized_distance);
}

/**
 * @brief 解析 ESR 发射源在当前周期的共享时序体制状态。
 * @param[in] dt_sec 周期步长（单位：s）。
 * @param[in] emitter 发射源真值。
 * @param[in] base_params ESR 统计检测基线参数。
 * @return 发射源在当前周期的统一时序体制状态。
 */
oneq::common::timing::ResolvedCycleTimingState ResolveEmitterTimingState(
    float dt_sec, const session::EsrSceneEmitter& emitter,
    const oneq::common::timing::StatisticalDetectionParams& base_params) {
  oneq::common::timing::CycleTimingBaseParams timing_base_params;
  timing_base_params.base_pulse_count = static_cast<int>(base_params.pulse_count);
  timing_base_params.base_prf_hz =
      emitter.pri_s > 0.0 ? static_cast<float>(1.0 / emitter.pri_s) : 0.0f;
  timing_base_params.cycle_dt_sec = dt_sec;
  timing_base_params.pri_s = emitter.pri_s;
  timing_base_params.integration_mode = base_params.integration_mode;
  return oneq::common::timing::ResolveCycleTimingState(
      timing_base_params, oneq::common::timing::CycleTimingControlAdjustments());
}

/**
 * @brief 构造当前周期接收机工作频段窗口。
 * @param[in] cycle_index 当前周期号。
 * @param[in] runtime_config 会话运行态配置。
 * @return 接收机频段 `[lower, upper]`，单位 Hz。
 */
std::pair<double, double> BuildReceiverWindow(
    std::uint32_t cycle_index, const extension::InterceptRuntimeConfig& runtime_config) {
  const std::vector<config::EsrTuningWindow>& tuning_plan =
      runtime_config.receiver_hardware.tuning_plan;
  if (!tuning_plan.empty()) {
    std::uint64_t total_dwell_cycles = 0U;
    for (const config::EsrTuningWindow& window : tuning_plan) {
      total_dwell_cycles += window.dwell_cycles;
    }
    if (total_dwell_cycles > 0U) {
      std::uint64_t phase = static_cast<std::uint64_t>(cycle_index) % total_dwell_cycles;
      for (const config::EsrTuningWindow& window : tuning_plan) {
        if (phase < window.dwell_cycles) {
          return std::make_pair(window.center_frequency_hz - 0.5 * window.bandwidth_hz,
                                window.center_frequency_hz + 0.5 * window.bandwidth_hz);
        }
        phase -= window.dwell_cycles;
      }
    }
  }
  if (runtime_config.use_fixed_receiver_window &&
      oneq::common::validation::IsFinite(runtime_config.receiver_lower_hz) &&
      oneq::common::validation::IsFinite(runtime_config.receiver_upper_hz) &&
      runtime_config.receiver_upper_hz > runtime_config.receiver_lower_hz) {
    return std::make_pair(runtime_config.receiver_lower_hz, runtime_config.receiver_upper_hz);
  }
  return std::make_pair(runtime_config.receiver_hardware.receiver_band_lower_hz,
                        runtime_config.receiver_hardware.receiver_band_upper_hz);
}

oneq::coordinate::EcefPositionM ResolvePlatformLinkPosition(const MutableEsrContext& ctx) {
  if (ctx.HasPlatformEcefKinematics()) {
    return ctx.GetPlatformPositionEcefM();
  }
  oneq::coordinate::EcefPositionM position;
  position.x_m = ctx.GetPlatformPose().position_m.x;
  position.y_m = ctx.GetPlatformPose().position_m.y;
  position.z_m = ctx.GetPlatformPose().position_m.z;
  return position;
}

oneq::coordinate::EcefVelocityMps ResolvePlatformLinkVelocity(const MutableEsrContext& ctx) {
  if (ctx.HasPlatformEcefKinematics()) {
    return ctx.GetPlatformVelocityEcefMps();
  }
  oneq::coordinate::EcefVelocityMps velocity;
  velocity.x_mps = ctx.GetPlatformPose().velocity_mps.x;
  velocity.y_mps = ctx.GetPlatformPose().velocity_mps.y;
  velocity.z_mps = ctx.GetPlatformPose().velocity_mps.z;
  return velocity;
}

oneq::coordinate::EcefPositionM ResolveEmitterLinkPosition(
    const session::EsrSceneEmitter& emitter, bool use_ecef) {
  if (use_ecef && emitter.has_ecef_kinematics) {
    return emitter.position_ecef_m;
  }
  oneq::coordinate::EcefPositionM position;
  position.x_m = emitter.pose.position_m.x;
  position.y_m = emitter.pose.position_m.y;
  position.z_m = emitter.pose.position_m.z;
  return position;
}

oneq::coordinate::EcefVelocityMps ResolveEmitterLinkVelocity(
    const session::EsrSceneEmitter& emitter, bool use_ecef) {
  if (use_ecef && emitter.has_ecef_kinematics) {
    return emitter.velocity_ecef_mps;
  }
  oneq::coordinate::EcefVelocityMps velocity;
  velocity.x_mps = emitter.pose.velocity_mps.x;
  velocity.y_mps = emitter.pose.velocity_mps.y;
  velocity.z_mps = emitter.pose.velocity_mps.z;
  return velocity;
}

bool TryMakeDirection(const oneq::coordinate::EcefPositionM& from,
                      const oneq::coordinate::EcefPositionM& to,
                      oneq::electromagnetics::RfEcefUnitVector* direction) {
  if (direction == nullptr) {
    return false;
  }
  const double dx = to.x_m - from.x_m;
  const double dy = to.y_m - from.y_m;
  const double dz = to.z_m - from.z_m;
  const double norm = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (!std::isfinite(norm) || norm <= 0.0) {
    return false;
  }
  direction->x = dx / norm;
  direction->y = dy / norm;
  direction->z = dz / norm;
  return true;
}

oneq::electromagnetics::RfReceiverSite BuildReceiverSite(
    const MutableEsrContext& ctx, const session::EsrSceneEmitter& target,
    const std::pair<double, double>& receiver_window) {
  const config::EsrHardwareConfig& hardware = ctx.GetRuntimeConfig().receiver_hardware;
  oneq::electromagnetics::RfReceiverSite receiver;
  receiver.entity_id = ctx.GetPlatformEntityId() == 0U
                           ? std::numeric_limits<std::uint64_t>::max()
                           : ctx.GetPlatformEntityId();
  receiver.position_ecef_m = ResolvePlatformLinkPosition(ctx);
  receiver.velocity_ecef_mps = ResolvePlatformLinkVelocity(ctx);
  receiver.window_duration_s = ctx.GetCycleDeltaTimeSec();
  receiver.center_frequency_hz = 0.5 * (receiver_window.first + receiver_window.second);
  receiver.bandwidth_hz = receiver_window.second - receiver_window.first;
  receiver.receiver_system_loss_db = std::max(0.0f, hardware.integrated_receive_loss_db);
  receiver.minimum_far_field_range_m = hardware.minimum_far_field_range_m;
  receiver.has_co_site_isolation = hardware.has_co_site_isolation;
  receiver.co_site_isolation_db = hardware.co_site_isolation_db;
  receiver.polarization = hardware.polarization;
  receiver.antenna.peak_gain_dbi = hardware.antenna_peak_gain_dbi;
  receiver.antenna.half_power_beamwidth_deg =
      std::max(1.0f, 0.5f * (hardware.beam_az_width_deg + hardware.beam_el_width_deg));
  receiver.antenna.sidelobe_level_db = hardware.antenna_sidelobe_level_db;
  receiver.antenna.backlobe_level_db = hardware.antenna_backlobe_level_db;
  receiver.antenna.cross_polarization_isolation_db =
      hardware.cross_polarization_isolation_db;
  const oneq::coordinate::EcefPositionM target_position =
      ResolveEmitterLinkPosition(target, ctx.HasPlatformEcefKinematics());
  TryMakeDirection(receiver.position_ecef_m, target_position,
                   &receiver.antenna.boresight_ecef_unit);
  return receiver;
}

oneq::electromagnetics::RfEmission BuildSceneRfEmission(
    const MutableEsrContext& ctx, const session::EsrSceneEmitter& emitter,
    float beam_overlap_ratio) {
  oneq::electromagnetics::RfEmission emission;
  emission.emission_id = emitter.emitter_id;
  emission.entity_id = emitter.emitter_id;
  emission.position_ecef_m =
      ResolveEmitterLinkPosition(emitter, ctx.HasPlatformEcefKinematics());
  emission.velocity_ecef_mps =
      ResolveEmitterLinkVelocity(emitter, ctx.HasPlatformEcefKinematics());
  emission.waveform_kind = oneq::electromagnetics::RfWaveformKind::kPulsed;
  TryMakeDirection(emission.position_ecef_m, ResolvePlatformLinkPosition(ctx),
                   &emission.antenna.boresight_ecef_unit);
  oneq::electromagnetics::RfEmissionSegment segment;
  segment.duration_s = ctx.GetCycleDeltaTimeSec();
  segment.center_frequency_hz = emitter.carrier_hz;
  segment.bandwidth_hz = emitter.bandwidth_hz;
  const double duty_ratio = emitter.pri_s > 0.0
                                ? std::min(1.0, emitter.pulse_width_s / emitter.pri_s)
                                : 1.0;
  segment.transmit_power_w =
      emitter.tx_power_w * duty_ratio * static_cast<double>(std::max(0.0f, beam_overlap_ratio));
  emission.segments.push_back(segment);
  return emission;
}

bool TryEvaluateSceneLink(const MutableEsrContext& ctx,
                          const session::EsrSceneEmitter& emitter, float beam_overlap_ratio,
                          const oneq::electromagnetics::RfReceiverSite& receiver,
                          oneq::electromagnetics::RfLinkResult* result) {
  oneq::electromagnetics::RfLinkEvaluationConfig link_config;
  link_config.additional_propagation_loss_db =
      std::max(0.0f, ctx.GetEnvironmentSnapshot().propagation_loss_db);
  return oneq::electromagnetics::TryEvaluateRfLink(
      BuildSceneRfEmission(ctx, emitter, beam_overlap_ratio), receiver, link_config, result);
}

bool TryResolveChannelPower(
    const oneq::electromagnetics::RfEmission& emission,
    const oneq::electromagnetics::RfLinkResult& tuned_receiver_link,
    const oneq::electromagnetics::RfReceiverSite& signal_channel_receiver,
    double* power_w) {
  if (power_w == nullptr ||
      emission.segments.size() != tuned_receiver_link.segment_results.size()) {
    return false;
  }
  double candidate = 0.0;
  for (std::size_t index = 0U; index < emission.segments.size(); ++index) {
    double frequency_overlap = 0.0;
    if (!oneq::electromagnetics::TryRfFrequencyOverlapFraction(
            emission.segments[index].center_frequency_hz,
            emission.segments[index].bandwidth_hz,
            signal_channel_receiver.center_frequency_hz,
            signal_channel_receiver.bandwidth_hz, &frequency_overlap)) {
      return false;
    }
    const oneq::electromagnetics::RfSegmentLinkResult& segment_link =
        tuned_receiver_link.segment_results[index];
    candidate += segment_link.received_power_before_overlap_w *
                 segment_link.time_overlap_fraction * frequency_overlap;
  }
  *power_w = candidate;
  return true;
}

/**
 * @brief 根据归一化循环相位解析当前周期激活波束并推进相位。
 * @param[in,out] scan_phase_cycles 当前完整扫描图循环相位。
 * @param[in] dt_sec 周期步长（单位：s）。
 * @param[in] scan_pattern_size 扫描序列长度。
 * @param[in] runtime_config 会话运行态配置。
 * @return 激活波束索引。
 */
std::size_t ResolveActiveBeamIndex(double* scan_phase_cycles, float dt_sec,
                                   std::size_t scan_pattern_size,
                                   const extension::InterceptRuntimeConfig& runtime_config) {
  if (scan_pattern_size == 0U || scan_phase_cycles == nullptr) {
    return 0U;
  }
  const double safe_dt_sec =
      (std::isfinite(dt_sec) != 0 && dt_sec > 0.0f) ? static_cast<double>(dt_sec) : 0.0;
  const double scan_rate_hz =
      (std::isfinite(runtime_config.scan_rate_hz) != 0 && runtime_config.scan_rate_hz > 0.0f)
          ? static_cast<double>(runtime_config.scan_rate_hz)
          : 1.0;
  const double normalized_phase =
      std::min(std::nextafter(1.0, 0.0), std::max(0.0, *scan_phase_cycles));
  const std::size_t beam_index = static_cast<std::size_t>(
      std::floor(normalized_phase * static_cast<double>(scan_pattern_size)));
  double next_phase = std::fmod(normalized_phase + scan_rate_hz * safe_dt_sec, 1.0);
  if (next_phase < 0.0) {
    next_phase += 1.0;
  }
  *scan_phase_cycles = next_phase;
  return beam_index;
}

/**
 * @brief 按观测条件映射观测质量等级。
 * @param[in] snr_db 观测信噪比（单位：dB）。
 * @param[in] is_jammed 是否受干扰显著影响。
 * @return 观测质量等级。
 */
session::EsrObservationQuality ClassifyObservationQuality(float snr_db) {
  if (snr_db >= 18.0f) {
    return session::EsrObservationQuality::kHigh;
  }
  if (snr_db >= 10.0f) {
    return session::EsrObservationQuality::kMedium;
  }
  return session::EsrObservationQuality::kLow;
}

/**
 * @brief 对真实观测施加欺骗式错分选扰动。
 * @param[in] deception_strength 欺骗强度。
 * @param[in] config 欺骗建模配置。
 * @param[in,out] rng 随机引擎。
 * @param[in,out] record 待扰动观测。
 */
void ApplyDeceptionConfusion(float deception_strength,
                             const extension::InterceptDeceptionModelConfig& config,
                             std::mt19937* rng, RawObservationRecord* record) {
  if (rng == nullptr || record == nullptr) {
    return;
  }
  const float strength = utils::Clamp01(deception_strength);
  const float aoa_std_deg = std::max(0.1f, config.aoa_confusion_std_deg);
  const float rf_ratio = std::max(0.0f, config.rf_confusion_ratio);
  const float pw_ratio = std::max(0.0f, config.pw_confusion_ratio);
  std::normal_distribution<float> az_noise_dist(0.0f, aoa_std_deg * strength);
  std::normal_distribution<float> el_noise_dist(0.0f, aoa_std_deg * 0.6f * strength);
  std::uniform_real_distribution<float> rf_ratio_dist(-rf_ratio, rf_ratio);
  std::uniform_real_distribution<float> pw_ratio_dist(-pw_ratio, pw_ratio);

  record->observation.aoa_az_deg += static_cast<double>(az_noise_dist(*rng));
  record->observation.aoa_el_deg += static_cast<double>(el_noise_dist(*rng));
  record->observation.rf_hz += static_cast<double>(record->observation.rf_hz) *
                               static_cast<double>(rf_ratio_dist(*rng) * strength);
  record->observation.pulse_width_s += static_cast<double>(record->observation.pulse_width_s) *
                                       static_cast<double>(pw_ratio_dist(*rng) * strength);
  record->observation.pulse_width_s = std::max(record->observation.pulse_width_s, 1.0e-9);
  record->deception_affected = true;
}

/**
 * @brief 生成欺骗式伪观测。
 * @param[in] template_record 模板观测。
 * @param[in] deception_strength 欺骗强度。
 * @param[in] config 欺骗建模配置。
 * @param[in,out] rng 随机引擎。
 * @param[in,out] next_observation_id 观测 ID 分配器。
 * @return 伪观测记录。
 */
RawObservationRecord BuildDeceptionRecord(const RawObservationRecord& template_record,
                                          float deception_strength,
                                          const extension::InterceptDeceptionModelConfig& config,
                                          std::mt19937* rng, std::uint64_t* next_observation_id) {
  RawObservationRecord record = template_record;
  if (next_observation_id != nullptr) {
    record.observation.observation_id = (*next_observation_id)++;
  }
  const float strength = utils::Clamp01(deception_strength);
  const float rf_ratio = std::max(0.0f, config.rf_confusion_ratio);
  std::normal_distribution<float> angle_dist(
      0.0f, std::max(0.5f, config.aoa_confusion_std_deg) * (0.5f + strength));
  std::uniform_real_distribution<float> rf_ratio_dist(-rf_ratio, rf_ratio);
  std::uniform_real_distribution<float> pw_shift_dist(-std::max(0.0f, config.pw_confusion_ratio),
                                                      std::max(0.0f, config.pw_confusion_ratio));
  std::uniform_real_distribution<float> snr_loss_dist(4.0f, 10.0f);
  if (rng != nullptr) {
    record.observation.aoa_az_deg += static_cast<double>(angle_dist(*rng));
    record.observation.aoa_el_deg += static_cast<double>(angle_dist(*rng) * 0.6f);
    record.observation.rf_hz += static_cast<double>(template_record.observation.rf_hz) *
                                static_cast<double>(rf_ratio_dist(*rng) * strength);
    record.observation.pulse_width_s +=
        static_cast<double>(template_record.observation.pulse_width_s) *
        static_cast<double>(pw_shift_dist(*rng));
    record.observation.pulse_width_s = std::max(record.observation.pulse_width_s, 1.0e-9);
    record.observation.snr_db -= static_cast<double>(snr_loss_dist(*rng));
  }
  record.observation.quality = session::EsrObservationQuality::kLow;
  record.observation.is_jammed = false;
  record.truth_emitter_id = 0U;
  record.truth_pri_s = 0.0;
  record.matched_truth = false;
  record.deception_affected = true;
  record.synthetic_false_alarm = true;
  return record;
}

}  // namespace

InterceptDetectionOutput InterceptDetectionExecutor::Execute(const MutableEsrContext& ctx,
                                                             std::mt19937& rng,
                                                             std::uint64_t& next_observation_id,
                                                             double* scan_phase_cycles) {
  InterceptDetectionOutput output;

  const intercept::ScanPatternConfig scan_pattern_config =
      InterceptComponentFactory::BuildScanPatternConfig(ctx.GetPipelineConfig());
  output.scan_pattern = intercept::ScanPatternGenerator::Generate(scan_pattern_config);
  if (output.scan_pattern.empty()) {
    output.scan_pattern.push_back(intercept::BeamPointingDeg());
  }
  const std::size_t active_beam_index =
      ResolveActiveBeamIndex(scan_phase_cycles, ctx.GetCycleDeltaTimeSec(),
                             output.scan_pattern.size(), ctx.GetRuntimeConfig());
  const intercept::BeamPointingDeg active_beam = output.scan_pattern[active_beam_index];

  const intercept::AngleErrorModelConfig angle_error_config =
      InterceptComponentFactory::BuildAngleErrorModelConfig(ctx.GetPipelineConfig());
  const std::pair<double, double> receiver_window =
      BuildReceiverWindow(ctx.GetCycleIndex(), ctx.GetRuntimeConfig());
  output.receiver_center_frequency_hz = 0.5 * (receiver_window.first + receiver_window.second);
  output.receiver_bandwidth_hz = receiver_window.second - receiver_window.first;
  const oneq::common::timing::StatisticalDetectionParams base_statistical_detection_params =
      ToTimingDetectionParams(ctx.GetPipelineConfig().statistical_detection);

  const auto& scene_emitters = ctx.GetSceneEmitters();
  const auto& config = ctx.GetPipelineConfig();

  output.raw_records.reserve(
      scene_emitters.size() *
      (1U + static_cast<std::size_t>(config.deception_model.max_false_observations_per_emitter)));
  for (std::size_t i = 0; i < scene_emitters.size(); ++i) {
    const session::EsrSceneEmitter& emitter = scene_emitters[i];
    if (!emitter.is_emitting || emitter.carrier_hz <= 0.0 || emitter.bandwidth_hz <= 0.0 ||
        emitter.tx_power_w <= 0.0) {
      continue;
    }
    ProcessSingleEmitter(emitter, active_beam, receiver_window, angle_error_config,
                         base_statistical_detection_params, ctx, rng, next_observation_id,
                         output.raw_records, &output.receiver_saturated);
    if (output.receiver_saturated) {
      output.raw_records.clear();
      break;
    }
  }

  PROJECT_LOG_DEBUG("[InterceptDetection] cycle_index={} raw_records={}", ctx.GetCycleIndex(),
                    output.raw_records.size());

  return output;
}

void InterceptDetectionExecutor::ProcessSingleEmitter(
    const session::EsrSceneEmitter& emitter, const intercept::BeamPointingDeg& active_beam,
    const std::pair<double, double>& receiver_window,
    const intercept::AngleErrorModelConfig& angle_error_config,
    const oneq::common::timing::StatisticalDetectionParams& base_statistical_detection_params,
    const MutableEsrContext& ctx, std::mt19937& rng, std::uint64_t& next_observation_id,
    std::vector<RawObservationRecord>& raw_records, bool* receiver_saturated) const {
  const auto& platform_pose = ctx.GetPlatformPose();
  const auto& env_snapshot = ctx.GetEnvironmentSnapshot();
  const auto& config = ctx.GetPipelineConfig();
  const auto& runtime_config = ctx.GetRuntimeConfig();
  std::uniform_real_distribution<float> uniform_01(0.0f, 1.0f);

  const float range_m = ComputeRangeM(platform_pose, emitter);
  const oneq::common::geometry::AzimuthElevationDeg target_look_angles =
      ApplyAntennaMountOffset(ComputeEmitterLookAngles(platform_pose, emitter), runtime_config);
  const float target_az_deg = target_look_angles.az_deg;
  const float target_el_deg = target_look_angles.el_deg;
  const float emitter_beam_overlap_ratio = ComputeEmitterBeamOverlapRatio(platform_pose, emitter);
  const bool emitter_beam_covered = emitter_beam_overlap_ratio > 0.0f;
  const oneq::common::timing::ResolvedCycleTimingState timing_state = ResolveEmitterTimingState(
      ctx.GetCycleDeltaTimeSec(), emitter, base_statistical_detection_params);
  const bool has_available_pulses = timing_state.effective_pulse_count > 0U;
  oneq::common::timing::StatisticalDetectionParams emitter_detection_params =
      base_statistical_detection_params;
  if (has_available_pulses) {
    emitter_detection_params.pulse_count = std::max(1U, timing_state.effective_pulse_count);
  }

  const oneq::electromagnetics::RfReceiverSite receiver =
      BuildReceiverSite(ctx, emitter, receiver_window);
  oneq::electromagnetics::RfLinkResult target_link;
  if (!TryEvaluateSceneLink(ctx, emitter, emitter_beam_overlap_ratio, receiver, &target_link)) {
    return;
  }
  const double received_power_w = target_link.total_received_power_w;

  oneq::electromagnetics::RfReceiverSite signal_channel_receiver = receiver;
  const double signal_channel_lower_hz =
      std::max(receiver_window.first, emitter.carrier_hz - 0.5 * emitter.bandwidth_hz);
  const double signal_channel_upper_hz =
      std::min(receiver_window.second, emitter.carrier_hz + 0.5 * emitter.bandwidth_hz);
  if (signal_channel_upper_hz <= signal_channel_lower_hz) {
    return;
  }
  signal_channel_receiver.center_frequency_hz =
      0.5 * (signal_channel_lower_hz + signal_channel_upper_hz);
  signal_channel_receiver.bandwidth_hz =
      signal_channel_upper_hz - signal_channel_lower_hz;

  double engineering_interference_power_w = 0.0;
  double front_end_other_input_power_w = 0.0;
  oneq::electromagnetics::RfLinkEvaluationConfig interference_link_config;
  interference_link_config.additional_propagation_loss_db =
      std::max(0.0f, env_snapshot.propagation_loss_db);
  for (const session::EsrSceneEmitter& other : ctx.GetSceneEmitters()) {
    if (!other.is_emitting || other.emitter_id == emitter.emitter_id || other.carrier_hz <= 0.0 ||
        other.bandwidth_hz <= 0.0 || other.tx_power_w <= 0.0) {
      continue;
    }
    const float other_beam_overlap = ComputeEmitterBeamOverlapRatio(platform_pose, other);
    const oneq::electromagnetics::RfEmission other_emission =
        BuildSceneRfEmission(ctx, other, other_beam_overlap);
    oneq::electromagnetics::RfLinkResult front_end_link;
    if (oneq::electromagnetics::TryEvaluateRfLink(
            other_emission, receiver, interference_link_config, &front_end_link)) {
      front_end_other_input_power_w += front_end_link.total_received_power_w;
      double channel_power_w = 0.0;
      if (TryResolveChannelPower(other_emission, front_end_link, signal_channel_receiver,
                                &channel_power_w)) {
        engineering_interference_power_w += channel_power_w;
      }
    }
  }
  if (env_snapshot.interference_mode ==
      oneq::electromagnetics::RfInterferenceMode::kEngineering) {
    for (const oneq::electromagnetics::RfEmission& engineering_emission :
         env_snapshot.engineering_emissions) {
      if (engineering_emission.entity_id == emitter.emitter_id) {
        continue;
      }
      oneq::electromagnetics::RfLinkResult front_end_link;
      if (oneq::electromagnetics::TryEvaluateRfLink(
              engineering_emission, receiver, interference_link_config, &front_end_link)) {
        front_end_other_input_power_w += front_end_link.total_received_power_w;
        double channel_power_w = 0.0;
        if (TryResolveChannelPower(engineering_emission, front_end_link,
                                  signal_channel_receiver, &channel_power_w)) {
          engineering_interference_power_w += channel_power_w;
        }
      }
    }
  }

  intercept::JammingAggregateResult jamming_result;
  double legacy_interference_power_w = 0.0;
  if (env_snapshot.interference_mode == oneq::electromagnetics::RfInterferenceMode::kLegacy) {
    jamming_result = intercept::JammingAggregator::Aggregate(
        env_snapshot.jammer_sources, emitter.carrier_hz, emitter.bandwidth_hz);
    legacy_interference_power_w =
        static_cast<double>(jamming_result.suppression_power_w) *
        static_cast<double>(std::max(0.0f, config.suppression_model.suppression_noise_scale));
  }
  const double interference_power_w =
      std::max(0.0, engineering_interference_power_w + legacy_interference_power_w);
  const double ambient_noise_power_w = std::max(
      (static_cast<double>(config.detection.receiver_noise_floor_w) +
       static_cast<double>(env_snapshot.clutter_noise_w)) *
          ResolveSpectrumOccupancyNoiseScale(env_snapshot.spectrum_occupancy_ratio),
      kNumericFloor);
  const double noise_power_w =
      std::max(ambient_noise_power_w + interference_power_w, kNumericFloor);
  const double total_receiver_input_power_w =
      received_power_w + front_end_other_input_power_w + legacy_interference_power_w;
  if (receiver_saturated != nullptr &&
      total_receiver_input_power_w >
          static_cast<double>(runtime_config.receiver_hardware.maximum_linear_input_power_w)) {
    *receiver_saturated = true;
    PROJECT_LOG_INFO("[InterceptDetection] receiver saturated input_w={:.6g} limit_w={:.6g}",
                     total_receiver_input_power_w,
                     runtime_config.receiver_hardware.maximum_linear_input_power_w);
    return;
  }
  const float static_threshold_snr_db = config.detection.minimum_snr_db;
  const float dynamic_threshold_snr_db =
      oneq::common::timing::ComputeDynamicThresholdSnrDb(noise_power_w, emitter_detection_params);
  const float detection_threshold_snr_db =
      config.statistical_detection.enable_statistical_detection
          ? std::max(static_threshold_snr_db, dynamic_threshold_snr_db)
          : static_threshold_snr_db;

  const intercept::BoundarySearchResult boundary_result = intercept::BoundarySearchSolver::Solve(
      1.0f, config.detection.max_detect_range_m, config.detection.boundary_resolution_m,
      config.detection.boundary_max_iterations, [&](float candidate_range_m) {
        const double candidate_received_power_w =
            received_power_w * static_cast<double>(range_m) * static_cast<double>(range_m) /
            std::max(static_cast<double>(candidate_range_m) * candidate_range_m, kNumericFloor);
        const float candidate_snr_db = ToDb(candidate_received_power_w / noise_power_w);
        return candidate_snr_db >= detection_threshold_snr_db;
      });
  const float snr_db = ToDb(received_power_w / noise_power_w);
  const float baseline_snr_db = ToDb(received_power_w / ambient_noise_power_w);

  intercept::InterceptGateInput gate_input;
  gate_input.line_of_sight = emitter_beam_covered;
  gate_input.target_az_deg = target_az_deg;
  gate_input.target_el_deg = target_el_deg;
  gate_input.beam_az_deg = active_beam.az_deg;
  gate_input.beam_el_deg = active_beam.el_deg;
  gate_input.beam_az_width_deg = std::max(1.0f, config.scan.az_step_deg);
  gate_input.beam_el_width_deg = std::max(1.0f, config.scan.el_step_deg);
  gate_input.receiver_lower_hz = receiver_window.first;
  gate_input.receiver_upper_hz = receiver_window.second;
  gate_input.signal_center_hz = emitter.carrier_hz;
  gate_input.signal_bandwidth_hz = emitter.bandwidth_hz;
  gate_input.range_m = range_m;
  gate_input.max_range_m =
      boundary_result.boundary_range_m > 0.0f ? boundary_result.boundary_range_m : 0.0f;
  gate_input.dynamic_range_margin_db = snr_db - detection_threshold_snr_db;
  gate_input.min_dynamic_range_margin_db = config.detection.min_dynamic_range_margin_db;
  gate_input.min_frequency_overlap_ratio = 0.05f;
  gate_input.beam_guard_factor = 1.5f;

  const intercept::InterceptGateDecision gate_decision =
      intercept::InterceptGate::Evaluate(gate_input);
  PROJECT_LOG_DEBUG(
      "[InterceptDetection] emitter_id={} range={:.0f} snr={:.1f} gate_passed={} "
      "beam_overlap={:.2f}",
      emitter.emitter_id, range_m, snr_db, gate_decision.passed, emitter_beam_overlap_ratio);
  const float effective_beamwidth_deg =
      std::max(1.0f, 0.5f * (gate_input.beam_az_width_deg + gate_input.beam_el_width_deg));
  const double measured_az_deg = static_cast<double>(target_az_deg) +
                                 static_cast<double>(intercept::AngleErrorModel::SampleErrorDeg(
                                     snr_db, effective_beamwidth_deg, &rng, angle_error_config));
  const double measured_el_deg = static_cast<double>(target_el_deg) +
                                 static_cast<double>(intercept::AngleErrorModel::SampleErrorDeg(
                                     snr_db, effective_beamwidth_deg, &rng, angle_error_config));
  const float jn_db = ToDb(interference_power_w / ambient_noise_power_w);
  const float snr_loss_db = baseline_snr_db - snr_db;
  const bool is_jammed =
      interference_power_w > 0.0 &&
      (jn_db >= runtime_config.receiver_hardware.jamming_jn_threshold_db ||
       snr_loss_db >= runtime_config.receiver_hardware.jamming_snr_loss_threshold_db);
  const float deception_probability = utils::Clamp01(
      jamming_result.deception_risk * jamming_result.deception_weighted_overlap_ratio *
      std::max(0.0f, config.deception_model.false_alarm_probability_scale));

  RawObservationRecord base_record;
  base_record.observation.observation_id = next_observation_id++;
  base_record.observation.timestamp_s =
      static_cast<double>(ctx.GetCycleIndex()) * static_cast<double>(ctx.GetCycleDeltaTimeSec());
  base_record.observation.aoa_az_deg = measured_az_deg;
  base_record.observation.aoa_el_deg = measured_el_deg;
  base_record.observation.rf_hz = emitter.carrier_hz;
  base_record.observation.bandwidth_hz = emitter.bandwidth_hz;
  base_record.observation.pri_s = emitter.pri_s;
  base_record.observation.pulse_width_s = emitter.pulse_width_s;
  const double linear_snr = std::pow(10.0, static_cast<double>(snr_db) / 10.0);
  const double relative_estimation_std =
      std::max(1.0e-6, std::min(0.25, 1.0 / std::sqrt(std::max(linear_snr, 1.0e-12))));
  base_record.observation.rf_std_hz =
      std::max(1.0, emitter.bandwidth_hz * relative_estimation_std);
  base_record.observation.bandwidth_std_hz =
      std::max(1.0, emitter.bandwidth_hz * relative_estimation_std);
  base_record.observation.pri_std_s =
      std::max(1.0e-12, emitter.pri_s * relative_estimation_std);
  base_record.observation.pulse_width_std_s =
      std::max(1.0e-12, emitter.pulse_width_s * relative_estimation_std);
  base_record.observation.amplitude_db = static_cast<double>(ToDb(received_power_w));
  base_record.observation.snr_db = static_cast<double>(snr_db);
  base_record.observation.quality = ClassifyObservationQuality(snr_db);
  base_record.observation.is_jammed = is_jammed;

  const std::uint32_t false_alarm_cap = config.deception_model.max_false_observations_per_emitter;
  bool detection_passed = gate_decision.passed && has_available_pulses;
  if (detection_passed && config.statistical_detection.enable_statistical_detection) {
    const float detection_probability =
        oneq::common::timing::ComputeStatisticalDetectionProbability(
            snr_db, detection_threshold_snr_db, emitter_detection_params);
    detection_passed = uniform_01(rng) < detection_probability;
  }
  if (!detection_passed) {
    for (std::uint32_t fake_index = 0U; fake_index < false_alarm_cap; ++fake_index) {
      if (uniform_01(rng) >= deception_probability) {
        continue;
      }
      raw_records.push_back(BuildDeceptionRecord(
          base_record, deception_probability, config.deception_model, &rng, &next_observation_id));
    }
    return;
  }

  RawObservationRecord record = base_record;
  record.truth_emitter_id = emitter.emitter_id;
  record.truth_pri_s = emitter.pri_s;
  record.matched_truth = true;

  const float confusion_probability = utils::Clamp01(
      deception_probability * std::max(0.0f, config.deception_model.confusion_probability_scale));
  if (uniform_01(rng) < confusion_probability) {
    ApplyDeceptionConfusion(deception_probability, config.deception_model, &rng, &record);
  }
  raw_records.push_back(record);

  for (std::uint32_t fake_index = 0U; fake_index < false_alarm_cap; ++fake_index) {
    if (uniform_01(rng) >= deception_probability) {
      continue;
    }
    raw_records.push_back(BuildDeceptionRecord(record, deception_probability,
                                               config.deception_model, &rng, &next_observation_id));
  }
}

}  // namespace pipeline
}  // namespace electronic_surveillance_radar
