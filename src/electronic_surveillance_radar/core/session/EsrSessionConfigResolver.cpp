#include "electronic_surveillance_radar/core/session/EsrSessionConfigResolver.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace electronic_surveillance_radar {
namespace core {
namespace session {
namespace internal {
namespace {

/**
 * @brief 判断双精度浮点值是否为有限数。
 * @param[in] value 输入值。
 * @return 有限数时返回 `true`。
 */
bool IsFinite(double value) { return std::isfinite(value) != 0; }

/**
 * @brief 判断单精度浮点值是否为有限数。
 * @param[in] value 输入值。
 * @return 有限数时返回 `true`。
 */
bool IsFinite(float value) { return std::isfinite(value) != 0; }

/**
 * @brief 归一化扫描起止边界，保证起点不大于终点。
 * @param[in,out] start 扫描起点角度（单位：deg）。
 * @param[in,out] end 扫描终点角度（单位：deg）。
 */
void NormalizeScanBounds(float* start, float* end) {
  if (start == nullptr || end == nullptr) {
    return;
  }
  if (*start > *end) {
    std::swap(*start, *end);
  }
}

/**
 * @brief 对统计检测参数施加模式映射。
 * @param[in] mode ESR 工作模式。
 * @param[in,out] config 统计检测参数。
 */
void ApplyWorkModeAdjustment(EsrWorkMode mode,
                             pipeline::InterceptStatisticalDetectionConfig* config) {
  if (config == nullptr) {
    return;
  }
  config->pulse_count = std::max<std::uint32_t>(1U, config->pulse_count);
  config->threshold_scale = IsFinite(config->threshold_scale) && config->threshold_scale > 0.0f
                                ? config->threshold_scale
                                : 1.0f;
  switch (mode) {
    case EsrWorkMode::kHgesm:
      config->pulse_count =
          std::min<std::uint32_t>(config->pulse_count * 4U, static_cast<std::uint32_t>(4096U));
      config->threshold_scale = std::max(0.1f, config->threshold_scale * 0.85f);
      break;
    case EsrWorkMode::kRwr:
      config->pulse_count = std::max<std::uint32_t>(1U, config->pulse_count / 2U);
      config->threshold_scale = std::max(0.1f, config->threshold_scale * 1.25f);
      break;
    case EsrWorkMode::kEsm:
    default:
      break;
  }
}

/**
 * @brief 按分层参数解析扫描边界。
 * @param[in] layered 分层参数。
 * @param[in] runtime_config 运行态参数。
 * @param[in,out] scan_config 扫描配置。
 */
void ResolveScanConfigFromLayered(const EsrLayeredConfig& layered,
                                  const pipeline::InterceptRuntimeConfig& runtime_config,
                                  pipeline::InterceptScanConfig* scan_config) {
  if (scan_config == nullptr) {
    return;
  }

  const float mount_az = runtime_config.antenna_mount_az_deg;
  const float mount_el = runtime_config.antenna_mount_el_deg;
  const EsrMissionControlConfig& mission = layered.mission;
  const EsrHardwareConfig& hardware = layered.hardware;

  scan_config->scan_start_pos = static_cast<int>(mission.scan_start_position);
  scan_config->scan_sequence = static_cast<int>(mission.scan_sequence);

  if (IsFinite(hardware.beam_az_width_deg) && hardware.beam_az_width_deg > 0.0f) {
    scan_config->az_step_deg = hardware.beam_az_width_deg;
  }
  if (IsFinite(hardware.beam_el_width_deg) && hardware.beam_el_width_deg > 0.0f) {
    scan_config->el_step_deg = hardware.beam_el_width_deg;
  }

  const bool explicit_bounds_valid =
      mission.use_explicit_scan_bounds && IsFinite(mission.scan_start_az_deg) &&
      IsFinite(mission.scan_end_az_deg) && IsFinite(mission.scan_start_el_deg) &&
      IsFinite(mission.scan_end_el_deg);
  if (explicit_bounds_valid) {
    float start_az = mission.scan_start_az_deg - mount_az;
    float end_az = mission.scan_end_az_deg - mount_az;
    float start_el = mission.scan_start_el_deg - mount_el;
    float end_el = mission.scan_end_el_deg - mount_el;
    NormalizeScanBounds(&start_az, &end_az);
    NormalizeScanBounds(&start_el, &end_el);
    scan_config->scan_start_az_deg = start_az;
    scan_config->scan_end_az_deg = end_az;
    scan_config->scan_start_el_deg = start_el;
    scan_config->scan_end_el_deg = end_el;
    return;
  }

  const bool has_center_az = IsFinite(mission.scan_center_az_deg);
  const bool has_center_el = IsFinite(mission.scan_center_el_deg);
  if (has_center_az) {
    float half_az_span =
        0.5f * std::fabs(scan_config->scan_end_az_deg - scan_config->scan_start_az_deg);
    if (IsFinite(hardware.az_scan_range_deg) && hardware.az_scan_range_deg > 0.0f) {
      half_az_span = 0.5f * hardware.az_scan_range_deg;
    }
    const float center_az = mission.scan_center_az_deg - mount_az;
    scan_config->scan_start_az_deg = center_az - half_az_span;
    scan_config->scan_end_az_deg = center_az + half_az_span;
  }
  if (has_center_el) {
    float half_el_span =
        0.5f * std::fabs(scan_config->scan_end_el_deg - scan_config->scan_start_el_deg);
    if (IsFinite(hardware.el_scan_range_deg) && hardware.el_scan_range_deg > 0.0f) {
      half_el_span = 0.5f * hardware.el_scan_range_deg;
    }
    const float center_el = mission.scan_center_el_deg - mount_el;
    scan_config->scan_start_el_deg = center_el - half_el_span;
    scan_config->scan_end_el_deg = center_el + half_el_span;
  }
  NormalizeScanBounds(&scan_config->scan_start_az_deg, &scan_config->scan_end_az_deg);
  NormalizeScanBounds(&scan_config->scan_start_el_deg, &scan_config->scan_end_el_deg);
}

}  // namespace

ResolvedEsrSessionConfig ResolveEsrSessionConfig(const EsrSessionConfig& config) {
  ResolvedEsrSessionConfig resolved;
  resolved.pipeline_config = config.pipeline_config;
  resolved.environment_config = config.environment_config;

  if (!config.enable_layered_config) {
    return resolved;
  }

  const EsrLayeredConfig& layered = config.layered_config;
  const EsrHardwareConfig& hardware = layered.hardware;
  const EsrMissionControlConfig& mission = layered.mission;

  resolved.runtime_config.sensor_enabled = mission.power_on;
  resolved.runtime_config.antenna_mount_az_deg =
      IsFinite(hardware.antenna_mount_az_deg) ? hardware.antenna_mount_az_deg : 0.0f;
  resolved.runtime_config.antenna_mount_el_deg =
      IsFinite(hardware.antenna_mount_el_deg) ? hardware.antenna_mount_el_deg : 0.0f;
  resolved.runtime_config.integrated_receive_loss_db =
      (IsFinite(hardware.integrated_receive_loss_db) && hardware.integrated_receive_loss_db > 0.0f)
          ? hardware.integrated_receive_loss_db
          : 0.0f;
  resolved.runtime_config.scan_rate_hz =
      (IsFinite(mission.scan_rate_hz) && mission.scan_rate_hz > 0.0f) ? mission.scan_rate_hz : 1.0f;

  if (IsFinite(hardware.receiver_band_lower_hz) && IsFinite(hardware.receiver_band_upper_hz) &&
      hardware.receiver_band_upper_hz > hardware.receiver_band_lower_hz) {
    resolved.runtime_config.use_fixed_receiver_window = true;
    resolved.runtime_config.receiver_lower_hz = hardware.receiver_band_lower_hz;
    resolved.runtime_config.receiver_upper_hz = hardware.receiver_band_upper_hz;
  }

  if (IsFinite(hardware.receiver_sensitivity_w) && hardware.receiver_sensitivity_w > 0.0f) {
    resolved.pipeline_config.detection.receiver_noise_floor_w = hardware.receiver_sensitivity_w;
  }

  ResolveScanConfigFromLayered(layered, resolved.runtime_config, &resolved.pipeline_config.scan);
  ApplyWorkModeAdjustment(mission.work_mode, &resolved.pipeline_config.statistical_detection);
  return resolved;
}

}  // namespace internal
}  // namespace session
}  // namespace core
}  // namespace electronic_surveillance_radar
