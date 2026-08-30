// Copyright 2026. All Rights Reserved.
//
// @file rir_measurement_error_test.cpp
// @brief 验证 RIR 测量误差模型（副本改写自 ar_signal_detection_test.cpp 的
//        MeasurementErrorModel 段；阶段 2-M M6）。

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <string>

#include "1q/remote_identification_radar/config/RirMissionConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "RirCycleInputTestUtil.h"
#include "RirSqliteTestUtil.h"
#include "remote_identification_radar/dwell/RirMeasurementErrorModel.h"
#include "remote_identification_radar/runtime/RirController.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using dwell::RirEffectiveBeamwidthDeg;
using dwell::RirMeasurementErrorModel;

/// @brief 高 SNR 下距离误差接近偏置项、角度误差小。
TEST(RirMeasurementErrorModelTest, HighSnrSmallErrors) {
  const auto state = RirMeasurementErrorModel::Compute(20.0f, RirEffectiveBeamwidthDeg{3.0f, 3.0f},
                                                       1.0e6f);
  EXPECT_NEAR(state.range_error_std_m, 27.5f, 0.5f);  // 0.5·150/10 + 20
  EXPECT_LT(state.angle_error_std_rad, 0.01f);
}

/// @brief 俯仰波束宽度加宽 → 等效角度标准差增大。
TEST(RirMeasurementErrorModelTest, ElevationBeamwidthAffectsEquivalentAngleStdDev) {
  const auto narrow =
      RirMeasurementErrorModel::Compute(13.0f, RirEffectiveBeamwidthDeg{3.0f, 3.0f}, 1.0e6f);
  const auto wide =
      RirMeasurementErrorModel::Compute(13.0f, RirEffectiveBeamwidthDeg{3.0f, 6.0f}, 1.0e6f);
  EXPECT_GT(wide.angle_error_std_rad, narrow.angle_error_std_rad);
  // 波束内距离误差不受角度维影响。
  EXPECT_FLOAT_EQ(wide.range_error_std_m, narrow.range_error_std_m);
}

/// @brief SNR 降低 → 距离与角度误差均增大。
TEST(RirMeasurementErrorModelTest, LowerSnrInflatesErrors) {
  const auto strong = RirMeasurementErrorModel::Compute(20.0f, RirEffectiveBeamwidthDeg{3.0f, 3.0f},
                                                        1.0e6f);
  const auto weak = RirMeasurementErrorModel::Compute(0.0f, RirEffectiveBeamwidthDeg{3.0f, 3.0f},
                                                     1.0e6f);
  EXPECT_GT(weak.range_error_std_m, strong.range_error_std_m);
  EXPECT_GT(weak.angle_error_std_rad, strong.angle_error_std_rad);
}

/// @brief 等效角度标准差 = az/el 两轴的 RMS 合成（各向同性时等于单轴）。
TEST(RirMeasurementErrorModelTest, EquivalentAngleIsRmsOfAxes) {
  const auto iso =
      RirMeasurementErrorModel::Compute(10.0f, RirEffectiveBeamwidthDeg{3.0f, 3.0f}, 1.0e6f);
  const auto az_only =
      RirMeasurementErrorModel::Compute(10.0f, RirEffectiveBeamwidthDeg{3.0f, 0.001f}, 1.0e6f);
  // el 波束极窄 → el 误差近似仅偏置项，等效值低于各向同性。
  EXPECT_LT(az_only.angle_error_std_rad, iso.angle_error_std_rad);
}

/// @brief 笛卡尔量测协瓦差雅可比仰角列 = ∂ENU/∂el = (−xz/rh, −yz/rh, rh)（模长=斜距），
///        与测试内独立构造的 J·diag(σr²,σθ²,σθ²)·Jᵀ 全矩阵一致（审计 A4：旧实现
///        仰角列整列多除 r²，高度轴贡献按 1/r⁴ 坍缩成"完美测量"）。
TEST(RirCartesianCovarianceTest, ElevationColumnMatchesInverseMapJacobian) {
  session::RirSceneTarget target;
  target.position_x = 3.0e6f;
  target.position_y = 4.0e6f;
  target.position_z = 1.0e5f;
  const float range_std_m = 50.0f;
  const float angle_std_rad = 0.002f;

  const tracking::RirMeasurementCovariance covariance =
      runtime::RirController::MakeCartesianMeasurementCovariance(target, range_std_m,
                                                                 angle_std_rad);

  // 独立构造期望值（double 复算，忽略函数内叠加的数值地板）。
  const double x = static_cast<double>(target.position_x);
  const double y = static_cast<double>(target.position_y);
  const double z = static_cast<double>(target.position_z);
  const double r = std::sqrt(x * x + y * y + z * z);
  const double rh = std::sqrt(x * x + y * y);
  const double sr = static_cast<double>(range_std_m);
  const double stheta = static_cast<double>(angle_std_rad);
  Eigen::Matrix3d jacobian;
  jacobian << x / r, -y, -x * z / rh,
              y / r,  x, -y * z / rh,
              z / r, 0.0, rh;
  const Eigen::Matrix3d expected =
      jacobian * Eigen::Vector3d(sr * sr, stheta * stheta, stheta * stheta).asDiagonal() *
      jacobian.transpose();

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      EXPECT_NEAR(covariance(row, col), expected(row, col),
                  1.0e-3 * std::abs(expected(row, col)) + 1.0e-2)
          << "(" << row << "," << col << ")";
    }
  }
  // 回归判别：远距离高度轴方差由仰角项主导（(σθ·rh)² ≈ 1e8 m² 量级），
  // 旧 1/r⁴ 坍缩口径下该元素坍缩到数值地板（~0.34 m²）。
  EXPECT_GT(covariance(2, 2), 1.0e6);
}

// ===== 误差模型输入口径（核查 6.1）：单脉冲 SNR + 10·log10(N) 积累折算 =====
//
// 经 RirController 全链反解：检测输出 snr_db 为单脉冲口径（common CFAR 两路径
// 同口径），误差模型须收到 单脉冲 SNR + 10·log10(N)（N=生效积累脉冲数）。
// 反解手段：目标置于 (x,0,0)（斜距-角度雅可比对角化：x 轴=距离项、y/z 轴=
// 角度项×x），同种子双档 N 运行的量测误差各轴比值 = 模型在两档等效 SNR 下的
// 标准差比值（首周期滤波初始化=量测位置，误差完全来自协方差采样且噪声向量相同）。

// schema v1.1 最小有效库（与 rir_feature_measurement_test 同款 fixture；仅用于
// 透出特征记录的 snr_db 观测窗，识别内容本身不是断言对象）。
constexpr const char* kCalibrationDatabaseSql = R"sql(
INSERT INTO meta VALUES
  ('schema_version','1.1'),
  ('database_id','rir-measurement-error-calibration-test'),
  ('version','1.0.0'),
  ('created_utc','2026-08-30T00:00:00Z'),
  ('polarization_channels','H,V'),
  ('polarization_energy_reference','range_propagation_antenna_compensated');
INSERT INTO units VALUES
  ('rcs','dBsm'),('speed','m/s'),('altitude','m'),('acceleration','m/s2'),
  ('turn_radius','m'),('polarization','dB'),('range','m');
INSERT INTO categories VALUES ('BALLISTIC','弹道目标',1.0);
INSERT INTO models VALUES ('BALLISTIC_EXAMPLE_A','BALLISTIC','弹道目标示例 A',1.0);
INSERT INTO profiles VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',6.0,50.0,NULL,NULL,NULL,NULL);
INSERT INTO rcs_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',-3.0,2.0,NULL,NULL,NULL);
)sql";

/// @brief 校准目标：y=z=0 → 视线 (0°,0°)、量测协方差对角（x=距离项，y/z=角度项）。
session::RirSceneTarget MakeCalibrationTarget() {
  session::RirSceneTarget target;
  target.external_target_id = 7U;
  target.target_name = "calibration-target";
  target.position_x = 20000.0f;
  target.velocity_x = 100.0f;
  // RCS 调至单脉冲 SNR ≈ 16.5 dB：N=1/N=10 双档均过检测器判决（含蒙特卡洛抽样），
  // 且角度误差的 SNR 项未被偏置项淹没（两档比值 ≈ 0.6，远离 1，旧口径比值恒 1
  // 可判别）。
  target.rcs = 2.5e-4f;
  // RCS 特征维网格（≥2 视角样本即有效），供特征记录透出 snr_db。
  session::RirAspectRcsSample sample_neg;
  sample_neg.aspect_az_deg = -5.0f;
  sample_neg.aspect_el_deg = 0.0f;
  sample_neg.rcs_dbsm = -3.0f;
  target.aspect_rcs_samples.push_back(sample_neg);
  session::RirAspectRcsSample sample_pos;
  sample_pos.aspect_az_deg = 5.0f;
  sample_pos.aspect_el_deg = 0.0f;
  sample_pos.rcs_dbsm = -3.0f;
  target.aspect_rcs_samples.push_back(sample_pos);
  return target;
}

runtime::RirController MakeCalibrationController(const std::string& database_path,
                                                 const config::RirHardwareConfig& hardware,
                                                 int pulse_count) {
  config::RirMissionConfig mission;
  mission.work_mode = config::RirWorkMode::kIdentify;
  config::RirPolicyConfig policy;
  policy.detection.gate_mode = config::RirDetectionGateMode::kDetectorGate;
  policy.detection.pulse_count = pulse_count;
  policy.lifecycle.confirm_hits = 1U;
  policy.recognition.enabled = true;
  policy.recognition.database_path = database_path;
  runtime::RirController controller;
  controller.SetHardware(hardware);
  controller.UpdateRuntime(mission, policy);
  return controller;
}

session::RirCycleInput MakeCalibrationInput(const session::RirSceneTarget& target) {
  session::RirCycleInput input;
  input.input_cycle_index = 1U;
  input.dt_sec = 0.5;
  input.sim_time_sec = 0.0f;
  SetDefaultTestPlatformEcef(&input);
  input.scene_targets.push_back(target);
  return input;
}

TEST(RirMeasurementErrorCalibrationTest, ErrorModelInputIncludesIntegrationGain) {
  const std::string database_path =
      WriteTempSqlite("rir_measurement_error_calibration.db",
                      std::string(kRecognitionSchemaSql) + kCalibrationDatabaseSql);
  ASSERT_FALSE(database_path.empty());

  // 缺省硬件（35 dBi / 4° 波束 / 4.5 MHz / 3 GHz）与控制器/模型两侧共用同源。
  const config::RirHardwareConfig hardware;
  const session::RirSceneTarget target = MakeCalibrationTarget();

  runtime::RirController controller_one = MakeCalibrationController(database_path, hardware, 1);
  runtime::RirController controller_ten = MakeCalibrationController(database_path, hardware, 10);

  session::RirOutputFrame frame_one;
  controller_one.RunCycle(MakeCalibrationInput(target), &frame_one, 1U,
                          config::RirAzimuthElevationDeg{0.0f, 0.0f});
  session::RirOutputFrame frame_ten;
  controller_ten.RunCycle(MakeCalibrationInput(target), &frame_ten, 1U,
                          config::RirAzimuthElevationDeg{0.0f, 0.0f});

  // 自证前提：两档均过检测门成轨，且特征记录透出单脉冲 SNR（N 不改写透出口径）。
  ASSERT_EQ(frame_one.recognition_outputs.size(), 1U);
  ASSERT_EQ(frame_ten.recognition_outputs.size(), 1U);
  ASSERT_EQ(frame_one.feature_measurements.size(), 1U);
  ASSERT_EQ(frame_ten.feature_measurements.size(), 1U);
  const float snr_single_db = frame_one.feature_measurements[0].snr_db;
  EXPECT_FLOAT_EQ(frame_ten.feature_measurements[0].snr_db, snr_single_db);
  // SNR 落在线性判别区（过高会使两档误差同被偏置项淹没，比值趋 1 失去判别力）。
  ASSERT_GT(snr_single_db, 6.0f);
  ASSERT_LT(snr_single_db, 20.0f);

  // 模型比值：同一模型在 单脉冲 SNR 与 +10·log10(10) 两档输入下的标准差比。
  const float wavelength_m = static_cast<float>(oneq::common::numerics::kLightSpeed) / 3.0e9f;
  const RirEffectiveBeamwidthDeg beamwidth =
      dwell::RirResolveEffectiveBeamwidth(hardware.antenna, wavelength_m);
  const dwell::RirMeasurementErrorState error_one = RirMeasurementErrorModel::Compute(
      snr_single_db, beamwidth, hardware.transmitter.bandwidth_hz);
  const dwell::RirMeasurementErrorState error_ten = RirMeasurementErrorModel::Compute(
      snr_single_db + 10.0f, beamwidth, hardware.transmitter.bandwidth_hz);

  // 反解：误差向量 = 首周期归属位置 − 真值（滤波初始化=采样量测位置）。
  const Eigen::Vector3f error_one_vec(
      static_cast<float>(controller_one.LatestTrackAttributions()[0].position_enu_x_m) -
          target.position_x,
      static_cast<float>(controller_one.LatestTrackAttributions()[0].position_enu_y_m) -
          target.position_y,
      static_cast<float>(controller_one.LatestTrackAttributions()[0].position_enu_z_m) -
          target.position_z);
  const Eigen::Vector3f error_ten_vec(
      static_cast<float>(controller_ten.LatestTrackAttributions()[0].position_enu_x_m) -
          target.position_x,
      static_cast<float>(controller_ten.LatestTrackAttributions()[0].position_enu_y_m) -
          target.position_y,
      static_cast<float>(controller_ten.LatestTrackAttributions()[0].position_enu_z_m) -
          target.position_z);

  // x 轴=距离项比值，y/z 轴=角度项比值（同种子噪声向量约去；容差吸收协方差
  // 数值地板 1e-6 m² 的二阶影响）。旧口径（直传单脉冲 SNR）比值恒为 1。
  EXPECT_NEAR(error_ten_vec.x() / error_one_vec.x(),
              error_ten.range_error_std_m / error_one.range_error_std_m, 1.0e-3f);
  EXPECT_NEAR(error_ten_vec.y() / error_one_vec.y(),
              error_ten.angle_error_std_rad / error_one.angle_error_std_rad, 1.0e-3f);
  EXPECT_NEAR(error_ten_vec.z() / error_one_vec.z(),
              error_ten.angle_error_std_rad / error_one.angle_error_std_rad, 1.0e-3f);
  // 方向性：积累折算后误差必须更小。
  EXPECT_LT(std::fabs(error_ten_vec.y()), std::fabs(error_one_vec.y()));
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
