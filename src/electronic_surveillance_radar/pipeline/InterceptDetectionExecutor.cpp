#include "electronic_surveillance_radar/pipeline/InterceptDetectionExecutor.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>

#include "common/geometry/GeometryTransform.h"
#include "common/numerics/SpectralNumerics.h"
#include "common/timing/TimingRegimeModel.h"
#include "electronic_surveillance_radar/utils/EsrSharedUtils.h"
#include "electronic_surveillance_radar/intercept/AngleErrorModel.h"
#include "electronic_surveillance_radar/intercept/BoundarySearchSolver.h"
#include "electronic_surveillance_radar/intercept/InterceptGate.h"
#include "electronic_surveillance_radar/intercept/JammingAggregator.h"
#include "electronic_surveillance_radar/intercept/ScanPatternGenerator.h"
#include "electronic_surveillance_radar/pipeline/InterceptComponentFactory.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace internal {

namespace {

/**
 * @brief 电磁传播计算中使用的圆周率常量。
 * @note 代码行为依据：接收功率计算使用自由空间路径损耗模型中的 `4πR/λ` 项。
 */
constexpr double kPi = 3.14159265358979323846;
/**
 * @brief 以米每秒表示的光速常量。
 * @note 代码行为依据：波长通过 `λ = c/f` 计算，用于路径损耗估计。
 */
constexpr double kLightSpeedMps = 299792458.0;
/**
 * @brief 数值稳定保护下限。
 * @note 代码行为依据：用于避免除零、`log10(0)` 与非正噪声功率。
 */
constexpr double kNumericFloor = 1.0e-18;
/**
 * @brief 判断双精度浮点是否为有限数。
 * @param[in] value 输入值。
 * @return 有限数时返回 `true`。
 */
bool IsFinite(double value) { return std::isfinite(value) != 0; }

/**
 * @brief 把综合接收损耗 dB 映射到接收功率比例。
 * @param[in] loss_db 综合接收损耗（单位：dB）。
 * @return 线性接收功率比例，范围 `(0, 1]`。
 */
double ComputeReceiveLossScale(float loss_db) {
  if (!std::isfinite(loss_db) || loss_db <= 0.0f) {
    return 1.0;
  }
  return std::pow(10.0, -static_cast<double>(loss_db) / 10.0);
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
oneq::internal::timing::IntegrationMode ToTimingIntegrationMode(extension::InterceptIntegrationMode mode) {
  if (mode == extension::InterceptIntegrationMode::kCoherent) {
    return oneq::internal::timing::IntegrationMode::kCoherent;
  }
  return oneq::internal::timing::IntegrationMode::kNonCoherent;
}

/**
 * @brief 把 ESR 统计检测配置映射为共享时序体制参数。
 * @param[in] config ESR 统计检测配置。
 * @return 共享时序体制参数。
 */
oneq::internal::timing::StatisticalDetectionParams ToTimingDetectionParams(
    const extension::InterceptStatisticalDetectionConfig& config) {
  oneq::internal::timing::StatisticalDetectionParams params;
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
float ComputeRangeM(const session::EsrPoseState& platform_pose,
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
oneq::internal::geometry::Vector3f ToGeometryVector(const session::EsrVector3f& vector) {
  oneq::internal::geometry::Vector3f result;
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
oneq::internal::geometry::EulerAnglesDeg ToGeometryEuler(const session::EsrEulerAngleDeg& angle) {
  oneq::internal::geometry::EulerAnglesDeg result;
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
oneq::internal::geometry::AzimuthElevationDeg ComputeEmitterLookAngles(
    const session::EsrPoseState& platform_pose, const session::EsrSceneEmitter& emitter_state) {
  return oneq::internal::geometry::ComputeRelativeLineOfSightAzEl(
      ToGeometryVector(platform_pose.position_m), ToGeometryEuler(platform_pose.attitude_deg),
      ToGeometryVector(emitter_state.pose.position_m));
}

/**
 * @brief 把平台机体参考系视线角转换为天线参考系视线角。
 * @param[in] look_angles 机体参考系方位/俯仰角。
 * @param[in] runtime_config 会话运行态配置。
 * @return 天线参考系方位/俯仰角。
 */
oneq::internal::geometry::AzimuthElevationDeg ApplyAntennaMountOffset(
    const oneq::internal::geometry::AzimuthElevationDeg& look_angles,
    const extension::InterceptRuntimeConfig& runtime_config) {
  oneq::internal::geometry::AzimuthElevationDeg adjusted = look_angles;
  adjusted.az_deg = oneq::internal::geometry::ComputeAzimuthDifferenceDeg(
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
float ComputeEmitterBeamOverlapRatio(const session::EsrPoseState& platform_pose,
                                     const session::EsrSceneEmitter& emitter_state) {
  if (IsLegacyDefaultBeamState(emitter_state.beam_state)) {
    return 1.0f;
  }

  const oneq::internal::geometry::AzimuthElevationDeg emitter_to_platform =
      oneq::internal::geometry::ComputeRelativeLineOfSightAzEl(
          ToGeometryVector(emitter_state.pose.position_m),
          ToGeometryEuler(emitter_state.pose.attitude_deg),
          ToGeometryVector(platform_pose.position_m));
  const double az_diff = std::fabs(static_cast<double>(
      oneq::internal::geometry::ComputeAzimuthDifferenceDeg(
          emitter_to_platform.az_deg,
          static_cast<float>(emitter_state.beam_state.center_az_deg))));
  const double el_diff =
      std::fabs(static_cast<double>(emitter_to_platform.el_deg) -
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
oneq::internal::timing::ResolvedCycleTimingState ResolveEmitterTimingState(
    float dt_sec, const session::EsrSceneEmitter& emitter,
    const oneq::internal::timing::StatisticalDetectionParams& base_params) {
  oneq::internal::timing::CycleTimingBaseParams timing_base_params;
  timing_base_params.base_pulse_count = static_cast<int>(base_params.pulse_count);
  timing_base_params.base_prf_hz =
      emitter.pri_s > 0.0 ? static_cast<float>(1.0 / emitter.pri_s) : 0.0f;
  timing_base_params.cycle_dt_sec = dt_sec;
  timing_base_params.pri_s = emitter.pri_s;
  timing_base_params.integration_mode = base_params.integration_mode;
  return oneq::internal::timing::ResolveCycleTimingState(
      timing_base_params, oneq::internal::timing::CycleTimingControlAdjustments());
}

/**
 * @brief 计算自由空间模型下的接收功率。
 * @param[in] tx_power_w 发射功率（单位：W）。
 * @param[in] carrier_hz 载频（单位：Hz）。
 * @param[in] range_m 距离（单位：m）。
 * @param[in] propagation_loss_db 额外传播损耗（单位：dB）。
 * @return 接收功率（单位：W）。
 */
double ComputeReceivedPowerW(double tx_power_w, double carrier_hz, float range_m,
                             float propagation_loss_db) {
  if (tx_power_w <= 0.0 || carrier_hz <= 0.0) {
    return 0.0;
  }
  const double safe_range = std::max(static_cast<double>(range_m), 1.0);
  const double wavelength = kLightSpeedMps / carrier_hz;
  const double fspl_linear = std::pow((4.0 * kPi * safe_range) / wavelength, 2.0);
  const double propagation_loss_linear =
      std::pow(10.0, static_cast<double>(std::max(0.0f, propagation_loss_db)) / 10.0);
  return tx_power_w / (fspl_linear * propagation_loss_linear);
}

/**
 * @brief 构造当前周期接收机工作频段窗口。
 * @param[in] cycle_index 当前周期号。
 * @param[in] runtime_config 会话运行态配置。
 * @return 接收机频段 `[lower, upper]`，单位 Hz。
 */
std::pair<double, double> BuildReceiverWindow(std::uint32_t cycle_index,
                                              const extension::InterceptRuntimeConfig& runtime_config) {
  if (runtime_config.use_fixed_receiver_window && IsFinite(runtime_config.receiver_lower_hz) &&
      IsFinite(runtime_config.receiver_upper_hz) &&
      runtime_config.receiver_upper_hz > runtime_config.receiver_lower_hz) {
    return std::make_pair(runtime_config.receiver_lower_hz, runtime_config.receiver_upper_hz);
  }
  struct BandWindow {
    double lower_hz;
    double upper_hz;
  };
  static const BandWindow kBandWindows[] = {
      {0.23e9, 1.0e9},  {1.0e9, 2.0e9},   {2.0e9, 4.0e9},   {4.0e9, 8.0e9},   {8.0e9, 12.0e9},
      {12.0e9, 18.0e9}, {18.0e9, 27.0e9}, {27.0e9, 40.0e9}, {40.0e9, 60.0e9}, {60.0e9, 100.0e9}};
  const std::size_t window_index =
      static_cast<std::size_t>(cycle_index) % (sizeof(kBandWindows) / sizeof(kBandWindows[0]));
  return std::make_pair(kBandWindows[window_index].lower_hz, kBandWindows[window_index].upper_hz);
}

/**
 * @brief 根据扫描数据率解析当前周期激活波束索引。
 * @param[in] cycle_index 当前周期号。
 * @param[in] dt_sec 周期步长（单位：s）。
 * @param[in] scan_pattern_size 扫描序列长度。
 * @param[in] runtime_config 会话运行态配置。
 * @return 激活波束索引。
 */
std::size_t ResolveActiveBeamIndex(std::uint32_t cycle_index, float dt_sec,
                                   std::size_t scan_pattern_size,
                                   const extension::InterceptRuntimeConfig& runtime_config) {
  if (scan_pattern_size == 0U) {
    return 0U;
  }
  const double safe_dt_sec =
      (std::isfinite(dt_sec) != 0 && dt_sec > 0.0f) ? static_cast<double>(dt_sec) : 1.0;
  const double scan_rate_hz =
      (std::isfinite(runtime_config.scan_rate_hz) != 0 && runtime_config.scan_rate_hz > 0.0f)
          ? static_cast<double>(runtime_config.scan_rate_hz)
          : 1.0;
  const double beam_advance_per_cycle = std::max(1.0, std::round(scan_rate_hz * safe_dt_sec));
  const std::uint64_t stride = static_cast<std::uint64_t>(std::max(1.0, beam_advance_per_cycle));
  const std::uint64_t beam_index = (static_cast<std::uint64_t>(cycle_index) * stride) %
                                   static_cast<std::uint64_t>(scan_pattern_size);
  return static_cast<std::size_t>(beam_index);
}

/**
 * @brief 按观测条件映射观测质量等级。
 * @param[in] snr_db 观测信噪比（单位：dB）。
 * @param[in] is_jammed 是否受干扰显著影响。
 * @return 观测质量等级。
 */
model::EsrObservationQuality ClassifyObservationQuality(float snr_db, bool is_jammed) {
  if (!is_jammed && snr_db >= 18.0f) {
    return model::EsrObservationQuality::kHigh;
  }
  if (snr_db >= 10.0f) {
    return model::EsrObservationQuality::kMedium;
  }
  return model::EsrObservationQuality::kLow;
}

/**
 * @brief 对真实观测施加欺骗式错分选扰动。
 * @param[in] deception_strength 欺骗强度。
 * @param[in] config 欺骗建模配置。
 * @param[in,out] rng 随机引擎。
 * @param[in,out] record 待扰动观测。
 */
void ApplyDeceptionConfusion(float deception_strength, const extension::InterceptDeceptionModelConfig& config,
                             std::mt19937* rng, internal::RawObservationRecord* record) {
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
  record->observation.pulse_width_s =
      std::max(record->observation.pulse_width_s, 1.0e-9);
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
internal::RawObservationRecord BuildDeceptionRecord(
    const internal::RawObservationRecord& template_record, float deception_strength,
    const extension::InterceptDeceptionModelConfig& config, std::mt19937* rng,
    std::uint64_t* next_observation_id) {
  internal::RawObservationRecord record = template_record;
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
    record.observation.pulse_width_s =
        std::max(record.observation.pulse_width_s, 1.0e-9);
    record.observation.snr_db -= static_cast<double>(snr_loss_dist(*rng));
  }
  record.observation.quality = model::EsrObservationQuality::kLow;
  record.observation.is_jammed = false;
  record.truth_emitter_id = 0U;
  record.truth_pri_s = 0.0;
  record.matched_truth = false;
  record.deception_affected = true;
  record.synthetic_false_alarm = true;
  return record;
}

}  // namespace

InterceptDetectionOutput InterceptDetectionExecutor::Execute(const extension::IEsrContext& ctx,
                                                             std::mt19937& rng,
                                                             std::uint64_t& next_observation_id) {
  InterceptDetectionOutput output;

  const intercept::ScanPatternConfig scan_pattern_config =
      InterceptComponentFactory::BuildScanPatternConfig(ctx.GetPipelineConfig());
  output.scan_pattern = intercept::ScanPatternGenerator::Generate(scan_pattern_config);
  if (output.scan_pattern.empty()) {
    output.scan_pattern.push_back(intercept::BeamPointingDeg());
  }
  const std::size_t active_beam_index =
      ResolveActiveBeamIndex(ctx.GetCycleIndex(), ctx.GetCycleDeltaTimeSec(),
                             output.scan_pattern.size(), ctx.GetRuntimeConfig());
  const intercept::BeamPointingDeg active_beam = output.scan_pattern[active_beam_index];

  const intercept::AngleErrorModelConfig angle_error_config =
      InterceptComponentFactory::BuildAngleErrorModelConfig(ctx.GetPipelineConfig());
  const std::pair<double, double> receiver_window =
      BuildReceiverWindow(ctx.GetCycleIndex(), ctx.GetRuntimeConfig());
  const double receive_loss_scale =
      ComputeReceiveLossScale(ctx.GetRuntimeConfig().integrated_receive_loss_db);
  const oneq::internal::timing::StatisticalDetectionParams base_statistical_detection_params =
      ToTimingDetectionParams(ctx.GetPipelineConfig().statistical_detection);

  const auto& scene_emitters = ctx.GetSceneEmitters();
  const auto& platform_pose = ctx.GetPlatformPose();
  const auto& env_snapshot = ctx.GetEnvironmentSnapshot();
  const auto& config = ctx.GetPipelineConfig();
  const auto& runtime_config = ctx.GetRuntimeConfig();

  output.raw_records.reserve(
      scene_emitters.size() *
      (1U + static_cast<std::size_t>(config.deception_model.max_false_observations_per_emitter)));
  std::uniform_real_distribution<float> uniform_01(0.0f, 1.0f);

  for (std::size_t i = 0; i < scene_emitters.size(); ++i) {
    const session::EsrSceneEmitter& emitter = scene_emitters[i];
    if (!emitter.is_emitting || emitter.carrier_hz <= 0.0 || emitter.bandwidth_hz <= 0.0 ||
        emitter.tx_power_w <= 0.0) {
      continue;
    }

    const float range_m = ComputeRangeM(platform_pose, emitter);
    const oneq::internal::geometry::AzimuthElevationDeg target_look_angles =
        ApplyAntennaMountOffset(ComputeEmitterLookAngles(platform_pose, emitter), runtime_config);
    const float target_az_deg = target_look_angles.az_deg;
    const float target_el_deg = target_look_angles.el_deg;
    const float emitter_beam_overlap_ratio = ComputeEmitterBeamOverlapRatio(platform_pose, emitter);
    const bool emitter_beam_covered = emitter_beam_overlap_ratio > 0.0f;
    const oneq::internal::timing::ResolvedCycleTimingState timing_state = ResolveEmitterTimingState(
        ctx.GetCycleDeltaTimeSec(), emitter, base_statistical_detection_params);
    const bool has_available_pulses = timing_state.effective_pulse_count > 0U;
    oneq::internal::timing::StatisticalDetectionParams emitter_detection_params =
        base_statistical_detection_params;
    if (has_available_pulses) {
      emitter_detection_params.pulse_count = std::max(1U, timing_state.effective_pulse_count);
    }

    const intercept::JammingAggregateResult jamming_result =
        intercept::JammingAggregator::Aggregate(env_snapshot.jammer_sources, emitter.carrier_hz,
                                                emitter.bandwidth_hz);
    const float suppression_noise_scale =
        std::max(0.0f, config.suppression_model.suppression_noise_scale);
    const double effective_suppression_power_w =
        static_cast<double>(jamming_result.suppression_power_w) *
        static_cast<double>(suppression_noise_scale);
    const double noise_power_w = std::max(
        static_cast<double>(config.detection.receiver_noise_floor_w) +
            static_cast<double>(env_snapshot.clutter_noise_w) + effective_suppression_power_w,
        kNumericFloor);
    const float static_threshold_snr_db = config.detection.min_detect_snr_db;
    const float dynamic_threshold_snr_db = oneq::internal::timing::ComputeDynamicThresholdSnrDb(
        noise_power_w, emitter_detection_params);
    const float detection_threshold_snr_db =
        config.statistical_detection.enable_statistical_detection
            ? std::max(static_threshold_snr_db, dynamic_threshold_snr_db)
            : static_threshold_snr_db;

    const intercept::BoundarySearchResult boundary_result = intercept::BoundarySearchSolver::Solve(
        1.0f, config.detection.max_detect_range_m, config.detection.boundary_resolution_m,
        config.detection.boundary_max_iterations, [&](float candidate_range_m) {
          const double candidate_received_power_w =
              ComputeReceivedPowerW(emitter.tx_power_w, emitter.carrier_hz, candidate_range_m,
                                    env_snapshot.propagation_loss_db) *
              static_cast<double>(emitter_beam_overlap_ratio) * receive_loss_scale;
          const float candidate_snr_db = ToDb(candidate_received_power_w / noise_power_w);
          return candidate_snr_db >= detection_threshold_snr_db;
        });
    const double received_power_w =
        ComputeReceivedPowerW(emitter.tx_power_w, emitter.carrier_hz, range_m,
                              env_snapshot.propagation_loss_db) *
        static_cast<double>(emitter_beam_overlap_ratio) * receive_loss_scale;
    const float snr_db = ToDb(received_power_w / noise_power_w);

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
    const float effective_beamwidth_deg =
        std::max(1.0f, 0.5f * (gate_input.beam_az_width_deg + gate_input.beam_el_width_deg));
    const double measured_az_deg =
        static_cast<double>(target_az_deg) +
        static_cast<double>(intercept::AngleErrorModel::SampleErrorDeg(
            snr_db, effective_beamwidth_deg, &rng, angle_error_config));
    const double measured_el_deg =
        static_cast<double>(target_el_deg) +
        static_cast<double>(intercept::AngleErrorModel::SampleErrorDeg(
            snr_db, effective_beamwidth_deg, &rng, angle_error_config));
    const bool is_jammed =
        effective_suppression_power_w >=
        static_cast<double>(std::max(0.0f, config.suppression_model.suppression_mark_threshold_w));
    const float deception_probability =
        utils::Clamp01(jamming_result.deception_risk * jamming_result.deception_weighted_overlap_ratio *
                std::max(0.0f, config.deception_model.false_alarm_probability_scale));

    internal::RawObservationRecord base_record;
    base_record.observation.observation_id = next_observation_id++;
    base_record.observation.timestamp_s =
        static_cast<double>(ctx.GetCycleIndex()) * static_cast<double>(ctx.GetCycleDeltaTimeSec());
    base_record.observation.aoa_az_deg = measured_az_deg;
    base_record.observation.aoa_el_deg = measured_el_deg;
    base_record.observation.rf_hz = emitter.carrier_hz;
    base_record.observation.pulse_width_s = emitter.pulse_width_s;
    base_record.observation.amplitude_db = static_cast<double>(ToDb(received_power_w));
    base_record.observation.snr_db = static_cast<double>(snr_db);
    base_record.observation.quality = ClassifyObservationQuality(snr_db, is_jammed);
    base_record.observation.is_jammed = is_jammed;

    const std::uint32_t false_alarm_cap = config.deception_model.max_false_observations_per_emitter;
    bool detection_passed = gate_decision.passed && has_available_pulses;
    if (detection_passed && config.statistical_detection.enable_statistical_detection) {
      const float detection_probability =
          oneq::internal::timing::ComputeStatisticalDetectionProbability(
              snr_db, detection_threshold_snr_db, emitter_detection_params);
      detection_passed = uniform_01(rng) < detection_probability;
    }
    if (!detection_passed) {
      for (std::uint32_t fake_index = 0U; fake_index < false_alarm_cap; ++fake_index) {
        if (uniform_01(rng) >= deception_probability) {
          continue;
        }
        output.raw_records.push_back(BuildDeceptionRecord(base_record, deception_probability,
                                                          config.deception_model, &rng,
                                                          &next_observation_id));
      }
      continue;
    }

    internal::RawObservationRecord record = base_record;
    record.truth_emitter_id = emitter.emitter_id;
    record.truth_pri_s = emitter.pri_s;
    record.matched_truth = true;

    const float confusion_probability = utils::Clamp01(
        deception_probability * std::max(0.0f, config.deception_model.confusion_probability_scale));
    if (uniform_01(rng) < confusion_probability) {
      ApplyDeceptionConfusion(deception_probability, config.deception_model, &rng, &record);
    }
    output.raw_records.push_back(record);

    for (std::uint32_t fake_index = 0U; fake_index < false_alarm_cap; ++fake_index) {
      if (uniform_01(rng) >= deception_probability) {
        continue;
      }
      output.raw_records.push_back(BuildDeceptionRecord(
          record, deception_probability, config.deception_model, &rng, &next_observation_id));
    }
  }

  return output;
}

}  // namespace internal
}  // namespace pipeline
}  // namespace electronic_surveillance_radar
