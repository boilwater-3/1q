/**
 * @file precision_evaluation_demo.cpp
 * @brief 精度评估层集成参考示例（需求 3.2.1.6.3：五项定位误差 + AHP 综合评分）。
 *
 * 场景为硬编码演示几何（复刻单测配方并扩为双目标）：
 *  - 双星：主星 (7e6,0,0)、辅星 (0,7e6,0)（ECEF 静止，速度/姿态零），各自扫描中心
 *    对准目标群（主星 ≈79.5°、辅星 ≈351.9°）；
 *  - 目标 1（key=1）：径向下降弹道（(8e6,6e6,0)，v=(−3200,−2400,0) m/s），供落点
 *    预测误差样本；目标 2（key=2）：邻近位置径向上升弹道，供发射点预测误差样本
 *    （两目标角距 ~2°，均在双星扫描带与宽视场内）；
 *  - 误差模型保持库默认（attitude σ=0.01°、固定 seed）——指标非零且可复现；
 *    SNR 门限放宽到 0.001 保证双星稳定检出。
 *
 * 每周期：调用方推进真值（p += v·dt）→ PrecisionEvaluationSession::Step 驱动
 * 内部双星 SBIRS + 融合（强制逐航迹滤波）+ 按间隔推演；结束 Summarize 汇总五指标
 * （mean/RMSE/P95/max）+ AHP 权重/一致性 + 综合得分。
 *
 * 运行：./build/<preset>/bin/precision_evaluation_demo [--cycles <n>]（默认 60）。
 * 验收日志流：配置加 -DONEQ_ENABLE_PRECISION_EVALUATION_LOG=ON 后，库内
 * [PrecisionEval] 事件（angular_error/dual_sat_fix/velocity_error/keypoint_error/
 * metric_summary/ahp_score）经 spdlog 默认 logger 打到 stdout——本示例不装文件 sink。
 */

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/precision_evaluation/PrecisionEvaluationConfig.h"
#include "1q/precision_evaluation/PrecisionEvaluationSession.h"
#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"

namespace pe = precision_evaluation;

namespace {

// GMST≈0 的儒略日（ECI≡ECEF，同 SBIRS 单测约定）。
constexpr double kUtcJulianDay = 2451544.2230698913;
constexpr std::uint32_t kDefaultCycles = 60U;

// 单星配置：扫描中心对准目标群，SNR 友好 + 误差模型库默认（非零、固定 seed）。
sbirs_sensor::config::SbirsSessionConfig SatelliteConfig(float scan_start_az_deg) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w = 1.0e-18f;
  config.hardware.integration_time_sec = 1.0f;
  config.mission.work_mode = sbirs_sensor::config::SbirsWorkMode::kSearchAndStare;
  config.mission.scan_start_az_deg = scan_start_az_deg;
  config.mission.scan_span_deg = 11.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.mission.wide_field_fov_az_deg = 20.0f;
  config.mission.wide_field_fov_el_deg = 20.0f;
  config.mission.narrow_field_fov_az_deg = 5.0f;
  config.mission.narrow_field_fov_el_deg = 5.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
  return config;
}

// 目标 1：径向下降（时域内落地 → 落点样本）。
pe::EvaluationTruthTarget MakeDescendingTarget() {
  pe::EvaluationTruthTarget truth;
  truth.key = 1U;
  truth.position_ecef_m = oneq::coordinate::EcefPositionM(8.0e6, 6.0e6, 0.0);
  truth.has_velocity = true;
  truth.velocity_ecef_m_per_s = oneq::coordinate::EcefVelocityMps(-3200.0, -2400.0, 0.0);
  truth.radiant_intensity_w_per_sr = 1.0e8;
  return truth;
}

// 目标 2：邻近位置径向上升（回推交地表 → 发射点样本；前向时域内不落地）。
pe::EvaluationTruthTarget MakeAscendingTarget() {
  pe::EvaluationTruthTarget truth;
  truth.key = 2U;
  truth.position_ecef_m = oneq::coordinate::EcefPositionM(8.2e6, 5.8e6, 0.0);
  truth.has_velocity = true;
  truth.velocity_ecef_m_per_s = oneq::coordinate::EcefVelocityMps(3266.0, 2309.0, 0.0);
  truth.radiant_intensity_w_per_sr = 1.0e8;
  return truth;
}

pe::PrecisionEvaluationConfig MakeEvaluationConfig() {
  pe::PrecisionEvaluationConfig config;
  config.satellite_a = SatelliteConfig(79.5f);   // 周期 1 扫描中心 ≈80.5°（正对目标群）
  config.satellite_b = SatelliteConfig(351.9f);  // 周期 1 扫描中心 ≈352.9°
  config.inference.prediction_horizon_sec = 1200.0;  // 径向入射 ~480 s 落地
  return config;
}

pe::DualSatEphemerisInput MakeEphemeris() {
  pe::DualSatEphemerisInput ephemeris;
  ephemeris.satellite_a_position_ecef_m = oneq::coordinate::EcefPositionM(7.0e6, 0.0, 0.0);
  ephemeris.satellite_b_position_ecef_m = oneq::coordinate::EcefPositionM(0.0, 7.0e6, 0.0);
  return ephemeris;
}

const char* MetricName(pe::PrecisionMetric metric) {
  switch (metric) {
    case pe::PrecisionMetric::kAngular:
      return "angular (deg)";
    case pe::PrecisionMetric::kDualSatFix:
      return "dual_sat_fix (m)";
    case pe::PrecisionMetric::kVelocity:
      return "velocity (m/s)";
    case pe::PrecisionMetric::kImpactPoint:
      return "impact_point (m)";
    case pe::PrecisionMetric::kLaunchPoint:
      return "launch_point (m)";
  }
  return "unknown";
}

void PrintReport(const pe::PrecisionEvaluationReport& report) {
  std::cout << "\n=== Precision Evaluation Report ===\n";
  std::cout << "metric               count          mean          rmse           p95           max\n";
  for (std::size_t i = 0U; i < pe::kPrecisionMetricCount; ++i) {
    const pe::ErrorMetricSummary& m = report.metrics[i];
    std::cout << MetricName(static_cast<pe::PrecisionMetric>(i)) << std::string(
                     22U - std::string(MetricName(static_cast<pe::PrecisionMetric>(i))).size(), ' ')
              << m.count << std::string(9U - std::to_string(m.count).size(), ' ')
              << m.mean << "  " << m.rmse << "  " << m.p95 << "  " << m.max << "\n";
  }
  std::cout << "\nahp: valid=" << (report.ahp_valid ? 1 : 0)
            << " consistent=" << (report.ahp.is_consistent ? 1 : 0)
            << " lambda_max=" << report.ahp.lambda_max
            << " ci=" << report.ahp.consistency_index
            << " cr=" << report.ahp.consistency_ratio << "\n";
  std::cout << "weights:";
  for (std::size_t i = 0U; i < pe::kPrecisionMetricCount; ++i) {
    std::cout << " " << report.ahp.weights[i];
  }
  std::cout << "\nscores (score = ref/(ref+rmse), contribution = w*score):\n";
  for (std::size_t i = 0U; i < pe::kPrecisionMetricCount; ++i) {
    std::cout << "  " << MetricName(static_cast<pe::PrecisionMetric>(i))
              << " ref=" << report.reference_errors[i]
              << " score=" << report.metric_scores[i]
              << " contribution=" << report.metric_contributions[i] << "\n";
  }
  std::cout << "composite score = " << report.composite_score << "\n";
}

}  // namespace

int main(int argc, char* argv[]) {
  std::uint32_t num_cycles = kDefaultCycles;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--cycles" && i + 1 < argc) {
      num_cycles = static_cast<std::uint32_t>(std::atoi(argv[++i]));
    } else {
      std::cerr << "Usage: " << argv[0] << " [--cycles <n>]\n";
      return 1;
    }
  }
  if (num_cycles == 0U) {
    std::cerr << "Invalid cycle count: must be > 0\n";
    return 1;
  }

  pe::PrecisionEvaluationSession session(MakeEvaluationConfig());
  const pe::DualSatEphemerisInput ephemeris = MakeEphemeris();
  std::vector<pe::EvaluationTruthTarget> truth = {MakeDescendingTarget(), MakeAscendingTarget()};

  std::cout << "precision_evaluation demo: dual-sat SBIRS + fusion + inference, "
            << num_cycles << " cycles x 1.0 s\n";
  for (std::uint32_t cycle = 1U; cycle <= num_cycles; ++cycle) {
    // 真值弹道由调用方推进（评估会话不拥有真值；同单测约定 p += v·dt）。
    for (auto& target : truth) {
      target.position_ecef_m.x_m += target.velocity_ecef_m_per_s.x_mps;
      target.position_ecef_m.y_m += target.velocity_ecef_m_per_s.y_mps;
      target.position_ecef_m.z_m += target.velocity_ecef_m_per_s.z_mps;
    }
    const pe::PrecisionEvaluationCycleResult result =
        session.Step(cycle, 1.0f, kUtcJulianDay, ephemeris, truth);
    std::cout << "cycle=" << cycle << " angular=" << result.angular.size()
              << " dual_sat=" << result.dual_sat.size()
              << " velocity=" << result.velocity.size()
              << " keypoints=" << result.keypoints.size() << "\n";
  }

  const pe::PrecisionEvaluationReport report = session.Summarize();
  PrintReport(report);

  // 自检：五指标均有样本、AHP 矩阵合法求解、综合分 ∈ (0,1]，否则视为链路断裂。
  if (!report.all_metrics_sampled || !report.ahp_valid || !(report.composite_score > 0.0) ||
      report.composite_score > 1.0) {
    std::cerr << "SMOKE FAILED: all_metrics_sampled=" << (report.all_metrics_sampled ? 1 : 0)
              << " ahp_valid=" << (report.ahp_valid ? 1 : 0)
              << " composite=" << report.composite_score << "\n";
    return 1;
  }
  return 0;
}
