/**
 * @file eos_look_angles_test.cpp
 * @brief 验证 EosLookAngles 对公共安装链的委托取角：零姿态一致性、姿态旋转、退化下限与斜距。
 */

#include <gtest/gtest.h>

#include <cmath>

#include "electro_optical_sensor/foundation/EosLookAngles.h"

namespace electro_optical_sensor {
namespace foundation {
namespace {

constexpr double kPi = 3.14159265358979323846;

oneq::coordinate::EulerAnglesDeg Attitude(double yaw_deg, double pitch_deg,
                                          double roll_deg) {
  oneq::coordinate::EulerAnglesDeg attitude;
  attitude.yaw_deg = yaw_deg;
  attitude.pitch_deg = pitch_deg;
  attitude.roll_deg = roll_deg;
  return attitude;
}

TEST(EosLookAnglesTest, RejectsNullOutputs) {
  float range_m = 0.0f;
  float azimuth_deg = 0.0f;
  EXPECT_FALSE(
      TryResolveEosLookAngles(100.0, 200.0, 300.0, Attitude(0.0, 0.0, 0.0), &range_m,
                              nullptr, &azimuth_deg));
}

TEST(EosLookAnglesTest, RejectsDegeneratePositionBelowNormFloor) {
  const float floor_m = EosLookAngleNormFloorM();
  float range_m = 0.0f;
  float azimuth_deg = 0.0f;
  float elevation_deg = 0.0f;

  EXPECT_FALSE(TryResolveEosLookAngles(0.0, 0.0, 0.0, Attitude(0.0, 0.0, 0.0),
                                       &range_m, &azimuth_deg, &elevation_deg));
  EXPECT_FALSE(TryResolveEosLookAngles(static_cast<double>(floor_m), 0.0, 0.0,
                                       Attitude(0.0, 0.0, 0.0), &range_m,
                                       &azimuth_deg, &elevation_deg));
  EXPECT_TRUE(TryResolveEosLookAngles(1.1e-6, 0.0, 0.0, Attitude(0.0, 0.0, 0.0),
                                      &range_m, &azimuth_deg, &elevation_deg));
}

TEST(EosLookAnglesTest, ZeroAttitudeMatchesDirectExtraction) {
  // 零姿态：体系 = ENU，方位/仰角与直接 atan2 提取一致（斜距 = ENU 模长）。
  const double x = 100.0;
  const double y = -200.0;
  const double z = 500.0;
  float range_m = 0.0f;
  float azimuth_deg = 0.0f;
  float elevation_deg = 0.0f;
  ASSERT_TRUE(TryResolveEosLookAngles(x, y, z, Attitude(0.0, 0.0, 0.0), &range_m,
                                      &azimuth_deg, &elevation_deg));
  const double expected_azimuth =
      std::atan2(y, x) * 180.0 / kPi;
  const double expected_elevation =
      std::atan2(z, std::sqrt(x * x + y * y)) * 180.0 / kPi;
  EXPECT_NEAR(azimuth_deg, expected_azimuth, 1.0e-4f);
  EXPECT_NEAR(elevation_deg, expected_elevation, 1.0e-4f);
  EXPECT_NEAR(range_m, std::sqrt(x * x + y * y + z * z), 1.0e-3f);
}

TEST(EosLookAnglesTest, AttitudeYawRotatesBodyAngles) {
  // 姿态 yaw=90°（Body->ENU）：ENU +x̂ 在机体系方位 -90°（ENU 旋入体系取角）。
  float range_m = 0.0f;
  float azimuth_deg = 0.0f;
  float elevation_deg = 0.0f;
  ASSERT_TRUE(TryResolveEosLookAngles(1000.0, 0.0, 0.0, Attitude(90.0, 0.0, 0.0),
                                      &range_m, &azimuth_deg, &elevation_deg));
  EXPECT_NEAR(azimuth_deg, -90.0f, 1.0e-4f);
  EXPECT_NEAR(elevation_deg, 0.0f, 1.0e-4f);
  EXPECT_NEAR(range_m, 1000.0f, 1.0e-3f);
}

TEST(EosLookAnglesTest, RoundTripPreservesDirection) {
  // 纯 yaw 姿态下提取的体系角重构 ENU 单位向量应与输入位置同向。
  const double x = 300.0;
  const double y = 400.0;
  const double z = -500.0;
  float range_m = 0.0f;
  float azimuth_deg = 0.0f;
  float elevation_deg = 0.0f;
  ASSERT_TRUE(TryResolveEosLookAngles(x, y, z, Attitude(35.0, 0.0, 0.0), &range_m,
                                      &azimuth_deg, &elevation_deg));

  const double az_rad = azimuth_deg * kPi / 180.0;
  const double el_rad = elevation_deg * kPi / 180.0;
  // 机体系单位向量经姿态旋回 ENU（Body->ENU = RotateLocalToEnu 语义）后应与输入同向。
  const double body_x = std::cos(el_rad) * std::cos(az_rad);
  const double body_y = std::cos(el_rad) * std::sin(az_rad);
  const double body_z = std::sin(el_rad);
  const double yaw_rad = 35.0 * kPi / 180.0;
  const double enu_x =
      std::cos(yaw_rad) * body_x - std::sin(yaw_rad) * body_y;
  const double enu_y =
      std::sin(yaw_rad) * body_x + std::cos(yaw_rad) * body_y;
  const double norm = std::sqrt(x * x + y * y + z * z);
  EXPECT_NEAR(enu_x, x / norm, 1.0e-4);
  EXPECT_NEAR(enu_y, y / norm, 1.0e-4);
  EXPECT_NEAR(body_z, z / norm, 1.0e-4);
}

}  // namespace
}  // namespace foundation
}  // namespace electro_optical_sensor
