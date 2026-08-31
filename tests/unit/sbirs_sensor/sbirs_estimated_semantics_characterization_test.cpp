// Copyright 2026. All Rights Reserved.
//
// @file sbirs_estimated_semantics_characterization_test.cpp
// @brief TARGET-OQ-3 证据：Estimated 与 Sensor-like 模式 raw output 的统计语义差异实测。
//
// 假设（docs/review/target_domain_p0_p1_decision_2026-08-17.md §4.1）：
//   Estimated 模式输出滤波后验角度（跨周期平滑估计），Sensor-like 输出独立子流的带误差量测。
// 证据门（以实测结构为准；相对关系弱断言，防脆性）：
//   1. Estimated 输出误差离散度 < Sensor-like（平滑降方差）；
//   2. Estimated 逐周期变化幅度至少比 Sensor-like 小一个数量级（滤波后验惯性）；
//   3. Estimated lag-1 自相关 > 0.8。
// 附带实测事实（RecordProperty 呈现，回写决策文档）：Sensor-like 误差亦非白噪声——
// 其 std 远大于配置角 σ，逐周期差分远小于 std，来自共模姿态/轨道偏差（慢变偏置）。
// 结论：估计层不得把 Estimated 输出当作独立单周期量测消费；Sensor-like 的 R 建模
// 亦须包含共模偏差项（对 P2 噪声通道设计的输入）。
// 数字经 RecordProperty 输出，回写决策文档指标签认表。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"

namespace sbirs_sensor {
namespace session {
namespace {

namespace output = ::sbirs_sensor::output;

SbirsVector3M Vector(double x, double y, double z) {
  SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

constexpr int kTotalCycles = 60;
constexpr int kWarmupCycles = 10;
constexpr double kTargetX = 8000000.0;
constexpr double kTargetVy = 500.0;  // m/s，60 周期方位漂移 ~0.21°，闭环跟踪可稳定保持

config::SbirsSessionConfig MakeSessionConfig(config::SbirsTrackingMode mode) {
  config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w = 1.0e-18f;
  config.hardware.integration_time_sec = 1.0f;
  config.mission.scan_start_az_deg = 359.0f;
  config.mission.scan_span_deg = 11.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  // 表征基线保持单帧量测口径（frame_rate×dt=1 帧）：本测试对照"滤波平滑 vs 逐帧
  // 独立噪声"的语义，多帧融合（高刷新建模，2026-08-31）会把对照侧方差一并压低，
  // 使门 1 失去区分度；多帧特性由 sbirs_error_model_test 融合用例覆盖。
  config.mission.frame_rate_hz = 1.0f;
  config.mission.wide_field_fov_az_deg = 20.0f;
  config.mission.wide_field_fov_el_deg = 20.0f;
  config.mission.narrow_field_fov_az_deg = 5.0f;
  config.mission.narrow_field_fov_el_deg = 5.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
  config.policy.tracking.tracking_mode = mode;
  // 三轴角误差 sigma 打开（RSS ≈ 0.087°），随机种子固定保证可复现。
  config.policy.error_model.orbit_sigma_deg = 0.05f;
  config.policy.error_model.attitude_sigma_deg = 0.05f;
  config.policy.error_model.fov_sigma_deg = 0.05f;
  config.policy.error_model.range_fraction_sigma = 0.0f;
  return config;
}

SbirsCycleInput MakeCycleInput(std::uint32_t cycle_index) {
  const double y = kTargetVy * static_cast<double>(cycle_index);
  SbirsSceneTarget target;
  target.target_id = 1U;
  target.target_name = "booster";
  target.position_ecef_m = Vector(kTargetX, y, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e8;
  target.velocity_ecef_m_per_s = Vector(0.0, kTargetVy, 0.0);
  target.has_velocity_ecef_m_per_s = true;
  return SbirsCycleInputBuilder()
      .WithCycleIndex(cycle_index)
      .WithDeltaTimeSec(1.0f)
      .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF，真值角可解析
      .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
      .WithSatelliteVelocity(SbirsVector3M{})
      .WithSatelliteAttitude(SbirsEulerAnglesDeg{})
      .AddTarget(target)
      .Build();
}

/// @brief 收集 NFOV 跟踪段（跳过捕获/预热）的方位误差序列（deg）。
/// @note 返回值函数内不能使用 ASSERT_*（其含裸 return），校验失败经 ok 出参上报。
std::vector<float> CollectAzimuthErrorDeg(config::SbirsTrackingMode mode, bool* ok) {
  *ok = true;
  SbirsSession session = SbirsSession::Create(MakeSessionConfig(mode));
  std::vector<float> errors;
  for (std::uint32_t cycle = 1U; cycle <= static_cast<std::uint32_t>(kTotalCycles); ++cycle) {
    const SbirsCycleResult result = session.StepWithResult(MakeCycleInput(cycle));
    if (result.status != SbirsCycleStatus::kCompleted ||
        result.output_frame.detections.size() != 1U) {
      *ok = false;
      return errors;
    }
    if (static_cast<int>(cycle) <= kWarmupCycles) {
      continue;
    }
    const auto& record = result.output_frame.detections.front();
    if (record.observation_stage != output::SbirsObservationStage::kNarrowFieldTrack) {
      *ok = false;
      return errors;
    }
    const double y = kTargetVy * static_cast<double>(cycle);
    const double truth_az_rad = std::atan2(y, kTargetX);
    const double error_deg =
        (static_cast<double>(record.azimuth_rad) - truth_az_rad) * 57.29577951308232;
    errors.push_back(static_cast<float>(error_deg));
  }
  return errors;
}

float StdOf(const std::vector<float>& values) {
  if (values.size() < 2U) {
    return 0.0f;
  }
  double mean = 0.0;
  for (float v : values) mean += v;
  mean /= static_cast<double>(values.size());
  double sum_sq = 0.0;
  for (float v : values) sum_sq += (v - mean) * (v - mean);
  return static_cast<float>(std::sqrt(sum_sq / static_cast<double>(values.size() - 1U)));
}

float StdOfSuccessiveDiff(const std::vector<float>& values) {
  std::vector<float> diff;
  for (std::size_t i = 1U; i < values.size(); ++i) {
    diff.push_back(values[i] - values[i - 1U]);
  }
  return StdOf(diff);
}

float LagOneAutocorrelation(const std::vector<float>& values) {
  if (values.size() < 3U) {
    return 0.0f;
  }
  double mean = 0.0;
  for (float v : values) mean += v;
  mean /= static_cast<double>(values.size());
  double num = 0.0;
  double den = 0.0;
  for (std::size_t i = 1U; i < values.size(); ++i) {
    num += (values[i] - mean) * (values[i - 1U] - mean);
  }
  for (float v : values) den += (v - mean) * (v - mean);
  return (den > 1.0e-12) ? static_cast<float>(num / den) : 0.0f;
}

TEST(SbirsEstimatedSemanticsCharacterizationTest, EstimatedOutputIsSmoothedNotIndependentNoise) {
  bool ok = false;
  const std::vector<float> estimated =
      CollectAzimuthErrorDeg(config::SbirsTrackingMode::kEstimated, &ok);
  ASSERT_TRUE(ok) << "Estimated session should track the whole arc";
  const std::vector<float> sensor_like =
      CollectAzimuthErrorDeg(config::SbirsTrackingMode::kSensorLikeTruthAssisted, &ok);
  ASSERT_TRUE(ok) << "Sensor-like session should track the whole arc";
  ASSERT_GT(estimated.size(), 10U);
  ASSERT_GT(sensor_like.size(), 10U);

  const float est_std = StdOf(estimated);
  const float sl_std = StdOf(sensor_like);
  const float est_diff = StdOfSuccessiveDiff(estimated);
  const float sl_diff = StdOfSuccessiveDiff(sensor_like);
  const float est_autocorr = LagOneAutocorrelation(estimated);
  const float sl_autocorr = LagOneAutocorrelation(sensor_like);

  RecordProperty("estimated_error_std_deg", std::to_string(est_std));
  RecordProperty("sensorlike_error_std_deg", std::to_string(sl_std));
  RecordProperty("estimated_diff_std_deg", std::to_string(est_diff));
  RecordProperty("sensorlike_diff_std_deg", std::to_string(sl_diff));
  RecordProperty("estimated_autocorr_lag1", std::to_string(est_autocorr));
  RecordProperty("sensorlike_autocorr_lag1", std::to_string(sl_autocorr));

  std::cout << "\n=== SBIRS Estimated vs Sensor-like output semantics ===\n"
            << "estimated : std=" << est_std << "deg diff_std=" << est_diff
            << "deg autocorr=" << est_autocorr << "\n"
            << "sensorlike: std=" << sl_std << "deg diff_std=" << sl_diff
            << "deg autocorr=" << sl_autocorr << "\n";

  // 门 1：平滑降方差。
  EXPECT_LT(est_std, sl_std) << "Estimated dispersion should be below Sensor-like";
  // 门 2：Estimated 逐周期变化至少比 Sensor-like 平滑一个数量级（滤波后验惯性）。
  EXPECT_LT(est_diff, 0.2f * sl_diff) << "Estimated per-cycle variation must be far smoother";
  // 门 3：Estimated 强自相关（后验惯性）。
  EXPECT_GT(est_autocorr, 0.8f);
}

}  // namespace
}  // namespace session
}  // namespace sbirs_sensor
