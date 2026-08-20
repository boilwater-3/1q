/**
 * @file esr_boresight_chain_test.cpp
 * @brief 验证 ESR 天线指向链薄适配：安装偏置符号契约、零姿态加法一致性、姿态复合与旋转合成。
 */

#include <gtest/gtest.h>

#include <cmath>

#include "electronic_surveillance_radar/pipeline/EsrBoresightChain.h"

namespace electronic_surveillance_radar {
namespace pipeline {
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

double AzimuthDegOf(const oneq::coordinate::Vector3d& direction) {
  return std::atan2(direction.y, direction.x) * 180.0 / kPi;
}

double ElevationDegOf(const oneq::coordinate::Vector3d& direction) {
  return std::atan2(direction.z, std::sqrt(direction.x * direction.x +
                                           direction.y * direction.y)) *
         180.0 / kPi;
}

TEST(EsrBoresightChainTest, ZeroAttitudeZeroMountKeepsAntennaPointing) {
  // 零姿态 + 零安装偏置：链路为恒等，天线系指向即 ENU 指向。
  const EsrBoresightChain chain(Attitude(0.0, 0.0, 0.0), 0.0, 0.0);

  for (const double azimuth_deg : {-160.0, -45.0, 0.0, 70.0, 175.0}) {
    for (const double elevation_deg : {-80.0, -15.0, 0.0, 30.0, 85.0}) {
      const oneq::coordinate::Vector3d enu =
          chain.EnuLosOfAntennaPointing(azimuth_deg, elevation_deg);
      EXPECT_NEAR(AzimuthDegOf(enu), azimuth_deg, 1.0e-9);
      EXPECT_NEAR(ElevationDegOf(enu), elevation_deg, 1.0e-9);
      EXPECT_NEAR(std::sqrt(enu.x * enu.x + enu.y * enu.y + enu.z * enu.z), 1.0,
                  1.0e-12);
    }
  }
}

TEST(EsrBoresightChainTest, PositiveMountOffsetsBoresightTowardPositiveBodyAngles) {
  // 符号契约：正安装偏置使光轴偏向机体系正方位/正仰角（与历史 beam + mount 同向）。
  const EsrBoresightChain azimuth_chain(Attitude(0.0, 0.0, 0.0), 30.0, 0.0);
  const oneq::coordinate::Vector3d azimuth_enu =
      azimuth_chain.EnuLosOfAntennaPointing(0.0, 0.0);
  EXPECT_NEAR(AzimuthDegOf(azimuth_enu), 30.0, 1.0e-9);
  EXPECT_NEAR(ElevationDegOf(azimuth_enu), 0.0, 1.0e-9);

  const EsrBoresightChain elevation_chain(Attitude(0.0, 0.0, 0.0), 0.0, 20.0);
  const oneq::coordinate::Vector3d elevation_enu =
      elevation_chain.EnuLosOfAntennaPointing(0.0, 0.0);
  EXPECT_NEAR(AzimuthDegOf(elevation_enu), 0.0, 1.0e-9);
  EXPECT_NEAR(ElevationDegOf(elevation_enu), 20.0, 1.0e-9);
}

TEST(EsrBoresightChainTest, ZeroAttitudeSingleAxisMountMatchesLegacyAddition) {
  // 零姿态 + 单轴安装偏置：与历史"波束角 + 安装偏置"角度加法严格一致
  //（纯方位：Rz 复合保持仰角；纯俯仰（方位 0）：Ry 复合等价角度相加）。
  const EsrBoresightChain azimuth_chain(Attitude(0.0, 0.0, 0.0), 15.0, 0.0);
  const oneq::coordinate::Vector3d azimuth_enu =
      azimuth_chain.EnuLosOfAntennaPointing(40.0, 8.0);
  EXPECT_NEAR(AzimuthDegOf(azimuth_enu), 55.0, 1.0e-9);
  EXPECT_NEAR(ElevationDegOf(azimuth_enu), 8.0, 1.0e-9);

  const EsrBoresightChain elevation_chain(Attitude(0.0, 0.0, 0.0), 0.0, 10.0);
  const oneq::coordinate::Vector3d elevation_enu =
      elevation_chain.EnuLosOfAntennaPointing(0.0, 25.0);
  EXPECT_NEAR(AzimuthDegOf(elevation_enu), 0.0, 1.0e-9);
  EXPECT_NEAR(ElevationDegOf(elevation_enu), 35.0, 1.0e-9);
}

TEST(EsrBoresightChainTest, AttitudeComposesWithMount) {
  // 姿态 yaw=90° ∘ 安装方位 10°：天线 az=0 → ENU 方位 100°
  //（R_enu_body · R_body_antenna · x̂ = Rz(90)·Rz(10)·x̂，AR 前向链同语义）。
  const EsrBoresightChain chain(Attitude(90.0, 0.0, 0.0), 10.0, 0.0);
  const oneq::coordinate::Vector3d enu = chain.EnuLosOfAntennaPointing(0.0, 0.0);
  EXPECT_NEAR(AzimuthDegOf(enu), 100.0, 1.0e-9);
  EXPECT_NEAR(ElevationDegOf(enu), 0.0, 1.0e-9);
}

TEST(EsrBoresightChainTest, CombinedMountComposesAsRotation) {
  // 双轴安装偏置按旋转合成（非角度加法）：v_body = Ry(-el_mount)·Rz(az_mount)·v_antenna。
  const double az_mount_deg = 30.0;
  const double el_mount_deg = 20.0;
  const EsrBoresightChain chain(Attitude(0.0, 0.0, 0.0), az_mount_deg, el_mount_deg);
  const oneq::coordinate::Vector3d enu = chain.EnuLosOfAntennaPointing(0.0, 0.0);

  const double az_rad = az_mount_deg * kPi / 180.0;
  const double el_rad = el_mount_deg * kPi / 180.0;
  const double expected_x = std::cos(el_rad) * std::cos(az_rad);
  const double expected_y = std::sin(az_rad);
  const double expected_z = std::sin(el_rad) * std::cos(az_rad);
  EXPECT_NEAR(enu.x, expected_x, 1.0e-12);
  EXPECT_NEAR(enu.y, expected_y, 1.0e-12);
  EXPECT_NEAR(enu.z, expected_z, 1.0e-12);
}

}  // namespace
}  // namespace pipeline
}  // namespace electronic_surveillance_radar
