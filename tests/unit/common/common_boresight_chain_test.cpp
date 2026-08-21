/**
 * @file common_boresight_chain_test.cpp
 * @brief 验证 common 安装矩阵合成链的恒等性、旋转语义、往返一致性与限位钳制。
 */

#include <gtest/gtest.h>

#include <cmath>

#include "common/geometry/BoresightChain.h"

namespace oneq {
namespace common {
namespace geometry {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

oneq::coordinate::Vector3d Vector(double x, double y, double z) {
  return oneq::coordinate::Vector3d(x, y, z);
}

TEST(CommonBoresightChainTest, ZeroIdentityPreservesReferenceAzEl) {
  // 零姿态 + 零安装角 + 零失准：链路为恒等变换，传感器系 az/el 与参考系 az/el 一致。
  const BoresightChain chain(EulerAnglesDeg{}, EulerAnglesDeg{});
  ASSERT_TRUE(chain.IsIdentity());

  float sensor_azimuth_deg = 0.0f;
  float sensor_elevation_deg = 0.0f;
  chain.SensorAzElOfReferenceVector(Vector(1.0, 0.0, 0.0), &sensor_azimuth_deg,
                                    &sensor_elevation_deg);
  EXPECT_FLOAT_EQ(sensor_azimuth_deg, 0.0f);
  EXPECT_FLOAT_EQ(sensor_elevation_deg, 0.0f);

  chain.SensorAzElOfReferenceVector(Vector(0.0, 1.0, 0.0), &sensor_azimuth_deg,
                                    &sensor_elevation_deg);
  EXPECT_NEAR(sensor_azimuth_deg, 90.0f, 1.0e-5f);
  EXPECT_NEAR(sensor_elevation_deg, 0.0f, 1.0e-5f);

  chain.SensorAzElOfReferenceVector(Vector(0.0, 0.0, 1.0), &sensor_azimuth_deg,
                                    &sensor_elevation_deg);
  EXPECT_NEAR(sensor_elevation_deg, 90.0f, 1.0e-5f);
}

TEST(CommonBoresightChainTest, AttitudeYawMapsSensorXToReferenceY) {
  // 姿态 yaw=90°（Body->Reference）：传感器 +x̂ 旋转到参考系 +ŷ。
  // 参考系由姿态参数定义（SBIRS 取 ECI；机载模块取 ENU 亦同构）。
  const EulerAnglesDeg attitude(90.0, 0.0, 0.0);
  const BoresightChain chain(attitude, EulerAnglesDeg{});
  ASSERT_FALSE(chain.IsIdentity());

  const oneq::coordinate::Vector3d reference_los =
      chain.ReferenceLosOfSensorPointing(0.0f, 0.0f);
  EXPECT_NEAR(reference_los.x, 0.0, 1.0e-9);
  EXPECT_NEAR(reference_los.y, 1.0, 1.0e-9);
  EXPECT_NEAR(reference_los.z, 0.0, 1.0e-9);

  float sensor_azimuth_deg = 0.0f;
  float sensor_elevation_deg = 0.0f;
  chain.SensorAzElOfReferenceVector(Vector(0.0, 1.0, 0.0), &sensor_azimuth_deg,
                                    &sensor_elevation_deg);
  EXPECT_NEAR(sensor_azimuth_deg, 0.0f, 1.0e-4f);
  EXPECT_NEAR(sensor_elevation_deg, 0.0f, 1.0e-4f);
}

TEST(CommonBoresightChainTest, MountPitchMapsSensorXToReferenceMinusZ) {
  // 安装角 pitch=90°：传感器 +x̂ 指向 body −ẑ（库约定正 pitch = 正仰角，
  // Ry(-90) 旋转使 x̂ → −ẑ）；零姿态下即参考系 −ẑ（el=-90°）。
  const EulerAnglesDeg mount(0.0, 90.0, 0.0);
  const BoresightChain chain(EulerAnglesDeg{}, mount);

  const oneq::coordinate::Vector3d reference_los =
      chain.ReferenceLosOfSensorPointing(0.0f, 0.0f);
  EXPECT_NEAR(reference_los.x, 0.0, 1.0e-9);
  EXPECT_NEAR(reference_los.y, 0.0, 1.0e-9);
  EXPECT_NEAR(reference_los.z, -1.0, 1.0e-9);
}

TEST(CommonBoresightChainTest, MountYawOffsetsSensorAzimuthInReferenceFrame) {
  // 机载场景复用（参考系=ENU）：安装角 yaw=+30°（链取 R_mount⁻¹，与姿态角方向相反）
  // 使参考系 +x̂ 在传感器系方位为 +30°；传感器 az=0 指向参考系方位 −30°。
  const EulerAnglesDeg mount(30.0, 0.0, 0.0);
  const BoresightChain chain(EulerAnglesDeg{}, mount);

  float sensor_azimuth_deg = 0.0f;
  float sensor_elevation_deg = 0.0f;
  chain.SensorAzElOfReferenceVector(Vector(1.0, 0.0, 0.0), &sensor_azimuth_deg,
                                    &sensor_elevation_deg);
  EXPECT_NEAR(sensor_azimuth_deg, 30.0f, 1.0e-4f);
  EXPECT_NEAR(sensor_elevation_deg, 0.0f, 1.0e-4f);

  const oneq::coordinate::Vector3d reference_los =
      chain.ReferenceLosOfSensorPointing(0.0f, 0.0f);
  const double reference_azimuth_rad = std::atan2(reference_los.y, reference_los.x);
  EXPECT_NEAR(reference_azimuth_rad * 180.0 / kPi, -30.0, 1.0e-4);
}

TEST(CommonBoresightChainTest, SensorAzElRoundTripIsIdentity) {
  const EulerAnglesDeg attitude(30.0, -15.0, 7.0);
  const EulerAnglesDeg mount(10.0, -25.0, 5.0);
  const BoresightChain chain(attitude, mount);

  for (const float azimuth_deg : {-160.0f, -60.0f, 0.0f, 45.0f, 170.0f}) {
    for (const float elevation_deg : {-60.0f, -20.0f, 0.0f, 30.0f, 70.0f}) {
      const oneq::coordinate::Vector3d reference_los =
          chain.ReferenceLosOfSensorPointing(azimuth_deg, elevation_deg);
      float round_trip_azimuth_deg = 0.0f;
      float round_trip_elevation_deg = 0.0f;
      chain.SensorAzElOfReferenceVector(reference_los, &round_trip_azimuth_deg,
                                        &round_trip_elevation_deg);
      EXPECT_NEAR(round_trip_azimuth_deg, azimuth_deg, 1.0e-3f);
      EXPECT_NEAR(round_trip_elevation_deg, elevation_deg, 1.0e-3f);
    }
  }
}

TEST(CommonBoresightChainTest, ReferenceVectorRoundTripPreservesDirection) {
  const EulerAnglesDeg attitude(120.0, 40.0, -10.0);
  const EulerAnglesDeg mount(-30.0, 15.0, 60.0);
  const BoresightChain chain(attitude, mount);

  // 非单位参考系向量：旋转保模长（R·Rᵀ=I 语义验证）。
  for (const double scale : {1.0, 2.5, -0.7}) {
    const oneq::coordinate::Vector3d input = Vector(3.0 * scale, -2.0 * scale, 4.0 * scale);
    const double input_norm =
        std::sqrt(input.x * input.x + input.y * input.y + input.z * input.z);
    float sensor_azimuth_deg = 0.0f;
    float sensor_elevation_deg = 0.0f;
    chain.SensorAzElOfReferenceVector(input, &sensor_azimuth_deg, &sensor_elevation_deg);
    const oneq::coordinate::Vector3d round_trip =
        chain.ReferenceLosOfSensorPointing(sensor_azimuth_deg, sensor_elevation_deg);
    EXPECT_NEAR(round_trip.x, input.x / input_norm, 1.0e-6);
    EXPECT_NEAR(round_trip.y, input.y / input_norm, 1.0e-6);
    EXPECT_NEAR(round_trip.z, input.z / input_norm, 1.0e-6);
  }
}

TEST(CommonBoresightChainTest, SensorPointingForDesiredReferenceLosInvertsChain) {
  // 惯性稳定反解原语：期望参考系光轴经链路反解为传感器系指向，
  // 应等于该光轴的传感器系 az/el。
  const EulerAnglesDeg attitude(45.0, 20.0, -30.0);
  const EulerAnglesDeg mount(5.0, -10.0, 0.0);
  const BoresightChain chain(attitude, mount);

  const double azimuth_rad = 300.0 * kPi / 180.0;
  const oneq::coordinate::Vector3d desired_reference_los =
      Vector(std::cos(azimuth_rad), std::sin(azimuth_rad), 0.0);
  float sensor_azimuth_deg = 0.0f;
  float sensor_elevation_deg = 0.0f;
  chain.SensorPointingForDesiredReferenceLos(desired_reference_los, &sensor_azimuth_deg,
                                             &sensor_elevation_deg);
  float direct_azimuth_deg = 0.0f;
  float direct_elevation_deg = 0.0f;
  chain.SensorAzElOfReferenceVector(desired_reference_los, &direct_azimuth_deg,
                                    &direct_elevation_deg);
  EXPECT_FLOAT_EQ(sensor_azimuth_deg, direct_azimuth_deg);
  EXPECT_FLOAT_EQ(sensor_elevation_deg, direct_elevation_deg);
}

TEST(CommonBoresightChainTest, ClampToScanLimitsCapsToWindowEdge) {
  AzimuthElevationLimitsDeg limits;
  limits.az_min_deg = -10.0f;
  limits.az_max_deg = 10.0f;
  limits.el_min_deg = -5.0f;
  limits.el_max_deg = 5.0f;

  float azimuth_deg = 45.0f;
  float elevation_deg = 60.0f;
  BoresightChain::ClampToScanLimits(limits, &azimuth_deg, &elevation_deg);
  EXPECT_FLOAT_EQ(azimuth_deg, 10.0f);
  EXPECT_FLOAT_EQ(elevation_deg, 5.0f);

  azimuth_deg = -45.0f;
  elevation_deg = -60.0f;
  BoresightChain::ClampToScanLimits(limits, &azimuth_deg, &elevation_deg);
  EXPECT_FLOAT_EQ(azimuth_deg, -10.0f);
  EXPECT_FLOAT_EQ(elevation_deg, -5.0f);

  azimuth_deg = 0.0f;
  elevation_deg = 0.0f;
  BoresightChain::ClampToScanLimits(limits, &azimuth_deg, &elevation_deg);
  EXPECT_FLOAT_EQ(azimuth_deg, 0.0f);
  EXPECT_FLOAT_EQ(elevation_deg, 0.0f);
}

TEST(CommonBoresightChainTest, NonZeroInputsAreNotIdentity) {
  const EulerAnglesDeg attitude(0.0, 5.0, 0.0);
  EXPECT_FALSE(BoresightChain(attitude, EulerAnglesDeg{}).IsIdentity());

  const EulerAnglesDeg mount(0.0, 0.0, 2.0);
  EXPECT_FALSE(BoresightChain(EulerAnglesDeg{}, mount).IsIdentity());
}

TEST(CommonBoresightChainTest, ZeroMisalignmentMatchesTwoArgConstructor) {
  // 3 参构造（零失准）与 2 参构造一致（2 参委托传零失准）。
  const EulerAnglesDeg attitude(12.0, 0.0, -4.0);
  const EulerAnglesDeg mount(0.0, 7.0, 0.0);
  const BoresightChain two_arg(attitude, mount);
  const BoresightChain three_arg(attitude, mount, EulerAnglesDeg{});
  const oneq::coordinate::Vector3d probe = Vector(0.6, -0.3, 0.8);
  const oneq::coordinate::Vector3d left = two_arg.RotateSensorToReference(probe);
  const oneq::coordinate::Vector3d right = three_arg.RotateSensorToReference(probe);
  EXPECT_NEAR(left.x, right.x, 1.0e-9);
  EXPECT_NEAR(left.y, right.y, 1.0e-9);
  EXPECT_NEAR(left.z, right.z, 1.0e-9);
  EXPECT_TRUE(three_arg.IsIdentity() == two_arg.IsIdentity());
}

TEST(CommonBoresightChainTest, MisalignmentYawRotatesSensorFrame) {
  // 失准与 mount 同语义（链取 R_misalign⁻¹）：misalignment yaw +90° 使传感器 +x̂ 指向
  // 参考系 -ŷ（绕 body z 反转 90°，与姿态 yaw 的方向相反）。
  const EulerAnglesDeg misalignment(90.0, 0.0, 0.0);
  const BoresightChain chain(EulerAnglesDeg{}, EulerAnglesDeg{}, misalignment);
  ASSERT_FALSE(chain.IsIdentity());
  const oneq::coordinate::Vector3d reference =
      chain.RotateSensorToReference(Vector(1.0, 0.0, 0.0));
  EXPECT_NEAR(reference.x, 0.0, 1.0e-9);
  EXPECT_NEAR(reference.y, -1.0, 1.0e-9);
  EXPECT_NEAR(reference.z, 0.0, 1.0e-9);
}

TEST(CommonBoresightChainTest, MisalignmentMakesChainNonIdentity) {
  const EulerAnglesDeg misalignment(0.0, 3.0, 0.0);
  EXPECT_FALSE(
      BoresightChain(EulerAnglesDeg{}, EulerAnglesDeg{}, misalignment).IsIdentity());
}

TEST(CommonBoresightChainTest, MisalignmentRoundTripPreservesDirection) {
  // R·Rᵀ = I：带失准的链 Reference->Sensor 往返保持方向与模长。
  const EulerAnglesDeg attitude(20.0, -6.0, 0.0);
  const EulerAnglesDeg mount(0.0, 0.0, 9.0);
  const EulerAnglesDeg misalignment(-15.0, 2.0, 0.0);
  const BoresightChain chain(attitude, mount, misalignment);
  const oneq::coordinate::Vector3d probe = Vector(0.4, 0.5, 0.3);
  const oneq::coordinate::Vector3d round_trip =
      chain.RotateReferenceToSensor(chain.RotateSensorToReference(probe));
  EXPECT_NEAR(round_trip.x, probe.x, 1.0e-9);
  EXPECT_NEAR(round_trip.y, probe.y, 1.0e-9);
  EXPECT_NEAR(round_trip.z, probe.z, 1.0e-9);
}

}  // namespace
}  // namespace geometry
}  // namespace common
}  // namespace oneq
