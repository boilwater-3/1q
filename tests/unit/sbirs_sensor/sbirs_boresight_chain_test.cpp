#include <gtest/gtest.h>

#include <cmath>

#include "sbirs_sensor/pipeline/SbirsBoresightChain.h"

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

sbirs_sensor::session::SbirsVector3M Vector(double x, double y, double z) {
  sbirs_sensor::session::SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

TEST(SbirsBoresightChainTest, ZeroIdentityPreservesEciAzEl) {
  // 零姿态 + 零安装角：链路为恒等变换，传感器系 az/el 与 ECI az/el 逐位一致。
  const sbirs_sensor::pipeline::SbirsBoresightChain chain(
      sbirs_sensor::session::SbirsEulerAnglesDeg{}, oneq::coordinate::EulerAnglesDeg{});
  ASSERT_TRUE(chain.IsIdentity());

  float sensor_azimuth_deg = 0.0f;
  float sensor_elevation_deg = 0.0f;
  chain.SensorAzElOfEciVector(Vector(1.0, 0.0, 0.0), &sensor_azimuth_deg, &sensor_elevation_deg);
  EXPECT_FLOAT_EQ(sensor_azimuth_deg, 0.0f);
  EXPECT_FLOAT_EQ(sensor_elevation_deg, 0.0f);

  chain.SensorAzElOfEciVector(Vector(0.0, 1.0, 0.0), &sensor_azimuth_deg, &sensor_elevation_deg);
  EXPECT_NEAR(sensor_azimuth_deg, 90.0f, 1.0e-5f);
  EXPECT_NEAR(sensor_elevation_deg, 0.0f, 1.0e-5f);

  chain.SensorAzElOfEciVector(Vector(0.0, 0.0, 1.0), &sensor_azimuth_deg, &sensor_elevation_deg);
  EXPECT_NEAR(sensor_elevation_deg, 90.0f, 1.0e-5f);
}

TEST(SbirsBoresightChainTest, YawRotationMapsSensorXToEciY) {
  // yaw=90°：传感器 +x̂ 旋转到 ECI +ŷ（传感器 az=0/el=0 对应的 ECI 方位为 90°）。
  sbirs_sensor::session::SbirsEulerAnglesDeg attitude;
  attitude.yaw_deg = 90.0;
  const sbirs_sensor::pipeline::SbirsBoresightChain chain(attitude,
                                                          oneq::coordinate::EulerAnglesDeg{});
  ASSERT_FALSE(chain.IsIdentity());

  const sbirs_sensor::session::SbirsVector3M eci_los = chain.EciLosOfSensorPointing(0.0f, 0.0f);
  EXPECT_NEAR(eci_los.x, 0.0, 1.0e-9);
  EXPECT_NEAR(eci_los.y, 1.0, 1.0e-9);
  EXPECT_NEAR(eci_los.z, 0.0, 1.0e-9);

  float sensor_azimuth_deg = 0.0f;
  float sensor_elevation_deg = 0.0f;
  chain.SensorAzElOfEciVector(Vector(0.0, 1.0, 0.0), &sensor_azimuth_deg, &sensor_elevation_deg);
  EXPECT_NEAR(sensor_azimuth_deg, 0.0f, 1.0e-4f);
  EXPECT_NEAR(sensor_elevation_deg, 0.0f, 1.0e-4f);
}

TEST(SbirsBoresightChainTest, MountPitchMapsSensorXToEciZ) {
  // 安装角 pitch=90°：传感器 +x̂ 指向 body −ẑ（库约定正 pitch = 正仰角，
  // Ry(-90) 旋转使 x̂ → −ẑ）；零姿态下即 ECI −ẑ（el=-90°）。
  oneq::coordinate::EulerAnglesDeg mount;
  mount.pitch_deg = 90.0;
  const sbirs_sensor::pipeline::SbirsBoresightChain chain(sbirs_sensor::session::SbirsEulerAnglesDeg{},
                                                          mount);

  const sbirs_sensor::session::SbirsVector3M eci_los = chain.EciLosOfSensorPointing(0.0f, 0.0f);
  EXPECT_NEAR(eci_los.x, 0.0, 1.0e-9);
  EXPECT_NEAR(eci_los.y, 0.0, 1.0e-9);
  EXPECT_NEAR(eci_los.z, -1.0, 1.0e-9);
}

TEST(SbirsBoresightChainTest, SensorAzElRoundTripIsIdentity) {
  const sbirs_sensor::session::SbirsEulerAnglesDeg attitude{30.0, -15.0, 7.0};
  oneq::coordinate::EulerAnglesDeg mount;
  mount.yaw_deg = 10.0;
  mount.pitch_deg = -25.0;
  mount.roll_deg = 5.0;
  const sbirs_sensor::pipeline::SbirsBoresightChain chain(attitude, mount);

  for (const float azimuth_deg : {-160.0f, -60.0f, 0.0f, 45.0f, 170.0f}) {
    for (const float elevation_deg : {-60.0f, -20.0f, 0.0f, 30.0f, 70.0f}) {
      const sbirs_sensor::session::SbirsVector3M eci_los =
          chain.EciLosOfSensorPointing(azimuth_deg, elevation_deg);
      float round_trip_azimuth_deg = 0.0f;
      float round_trip_elevation_deg = 0.0f;
      chain.SensorAzElOfEciVector(eci_los, &round_trip_azimuth_deg, &round_trip_elevation_deg);
      EXPECT_NEAR(round_trip_azimuth_deg, azimuth_deg, 1.0e-3f);
      EXPECT_NEAR(round_trip_elevation_deg, elevation_deg, 1.0e-3f);
    }
  }
}

TEST(SbirsBoresightChainTest, EciVectorRoundTripPreservesDirection) {
  const sbirs_sensor::session::SbirsEulerAnglesDeg attitude{120.0, 40.0, -10.0};
  oneq::coordinate::EulerAnglesDeg mount;
  mount.yaw_deg = -30.0;
  mount.pitch_deg = 15.0;
  mount.roll_deg = 60.0;
  const sbirs_sensor::pipeline::SbirsBoresightChain chain(attitude, mount);

  // 非单位 ECI 向量：旋转保模长（R·Rᵀ=I 语义验证）。
  for (const double scale : {1.0, 2.5, -0.7}) {
    const sbirs_sensor::session::SbirsVector3M input = Vector(3.0 * scale, -2.0 * scale, 4.0 * scale);
    const double input_norm =
        std::sqrt(input.x * input.x + input.y * input.y + input.z * input.z);
    float sensor_azimuth_deg = 0.0f;
    float sensor_elevation_deg = 0.0f;
    chain.SensorAzElOfEciVector(input, &sensor_azimuth_deg, &sensor_elevation_deg);
    const sbirs_sensor::session::SbirsVector3M round_trip =
        chain.EciLosOfSensorPointing(sensor_azimuth_deg, sensor_elevation_deg);
    EXPECT_NEAR(round_trip.x, input.x / input_norm, 1.0e-6);
    EXPECT_NEAR(round_trip.y, input.y / input_norm, 1.0e-6);
    EXPECT_NEAR(round_trip.z, input.z / input_norm, 1.0e-6);
  }
}

TEST(SbirsBoresightChainTest, SensorPointingForDesiredEciLosInvertsChain) {
  // 惯性稳定反解：期望 ECI 光轴经链路反解为传感器系指向，应等于该光轴的传感器系 az/el。
  const sbirs_sensor::session::SbirsEulerAnglesDeg attitude{45.0, 20.0, -30.0};
  oneq::coordinate::EulerAnglesDeg mount;
  mount.yaw_deg = 5.0;
  mount.pitch_deg = -10.0;
  const sbirs_sensor::pipeline::SbirsBoresightChain chain(attitude, mount);

  const double azimuth_rad = 300.0 * kPi / 180.0;
  const sbirs_sensor::session::SbirsVector3M desired_eci_los =
      Vector(std::cos(azimuth_rad), std::sin(azimuth_rad), 0.0);
  float sensor_azimuth_deg = 0.0f;
  float sensor_elevation_deg = 0.0f;
  chain.SensorPointingForDesiredEciLos(desired_eci_los, &sensor_azimuth_deg,
                                       &sensor_elevation_deg);
  float direct_azimuth_deg = 0.0f;
  float direct_elevation_deg = 0.0f;
  chain.SensorAzElOfEciVector(desired_eci_los, &direct_azimuth_deg, &direct_elevation_deg);
  EXPECT_FLOAT_EQ(sensor_azimuth_deg, direct_azimuth_deg);
  EXPECT_FLOAT_EQ(sensor_elevation_deg, direct_elevation_deg);
}

TEST(SbirsBoresightChainTest, ClampToScanLimitsCapsToWindowEdge) {
  sbirs_sensor::config::SbirsScanLimitsDeg limits;
  limits.az_min_deg = -10.0f;
  limits.az_max_deg = 10.0f;
  limits.el_min_deg = -5.0f;
  limits.el_max_deg = 5.0f;

  float azimuth_deg = 45.0f;
  float elevation_deg = 60.0f;
  sbirs_sensor::pipeline::SbirsBoresightChain::ClampToScanLimits(limits, &azimuth_deg,
                                                                 &elevation_deg);
  EXPECT_FLOAT_EQ(azimuth_deg, 10.0f);
  EXPECT_FLOAT_EQ(elevation_deg, 5.0f);

  azimuth_deg = -45.0f;
  elevation_deg = -60.0f;
  sbirs_sensor::pipeline::SbirsBoresightChain::ClampToScanLimits(limits, &azimuth_deg,
                                                                 &elevation_deg);
  EXPECT_FLOAT_EQ(azimuth_deg, -10.0f);
  EXPECT_FLOAT_EQ(elevation_deg, -5.0f);

  azimuth_deg = 0.0f;
  elevation_deg = 0.0f;
  sbirs_sensor::pipeline::SbirsBoresightChain::ClampToScanLimits(limits, &azimuth_deg,
                                                                 &elevation_deg);
  EXPECT_FLOAT_EQ(azimuth_deg, 0.0f);
  EXPECT_FLOAT_EQ(elevation_deg, 0.0f);
}

TEST(SbirsBoresightChainTest, NonZeroInputsAreNotIdentity) {
  sbirs_sensor::session::SbirsEulerAnglesDeg attitude;
  attitude.pitch_deg = 5.0;
  EXPECT_FALSE(
      sbirs_sensor::pipeline::SbirsBoresightChain(attitude, oneq::coordinate::EulerAnglesDeg{})
          .IsIdentity());

  oneq::coordinate::EulerAnglesDeg mount;
  mount.roll_deg = 2.0;
  EXPECT_FALSE(sbirs_sensor::pipeline::SbirsBoresightChain(
                   sbirs_sensor::session::SbirsEulerAnglesDeg{}, mount)
                   .IsIdentity());
}

TEST(SbirsBoresightChainTest, ZeroMisalignmentMatchesTwoArgConstructor) {
  // 阶段 3：3 参构造（零失准）与 2 参构造逐位一致（2 参委托传零失准）。
  sbirs_sensor::session::SbirsEulerAnglesDeg attitude;
  attitude.yaw_deg = 12.0;
  attitude.roll_deg = -4.0;
  oneq::coordinate::EulerAnglesDeg mount;
  mount.pitch_deg = 7.0;
  const sbirs_sensor::pipeline::SbirsBoresightChain two_arg(attitude, mount);
  const sbirs_sensor::pipeline::SbirsBoresightChain three_arg(attitude, mount,
                                                              oneq::coordinate::EulerAnglesDeg{});
  const sbirs_sensor::session::SbirsVector3M probe = Vector(0.6, -0.3, 0.8);
  const sbirs_sensor::session::SbirsVector3M left = two_arg.RotateSensorToEci(probe);
  const sbirs_sensor::session::SbirsVector3M right = three_arg.RotateSensorToEci(probe);
  EXPECT_NEAR(left.x, right.x, 1.0e-9);
  EXPECT_NEAR(left.y, right.y, 1.0e-9);
  EXPECT_NEAR(left.z, right.z, 1.0e-9);
  EXPECT_TRUE(three_arg.IsIdentity() == two_arg.IsIdentity());
}

TEST(SbirsBoresightChainTest, MisalignmentYawRotatesSensorFrame) {
  // 失准与 mount 同语义（链取 R_misalign⁻¹）：misalignment yaw +90° 使传感器 +x̂ 指向
  // ECI -ŷ（绕 body z 反转 90°，与姿态 yaw 的方向相反）。
  oneq::coordinate::EulerAnglesDeg misalignment;
  misalignment.yaw_deg = 90.0;
  const sbirs_sensor::pipeline::SbirsBoresightChain chain(
      sbirs_sensor::session::SbirsEulerAnglesDeg{}, oneq::coordinate::EulerAnglesDeg{},
      misalignment);
  ASSERT_FALSE(chain.IsIdentity());
  const sbirs_sensor::session::SbirsVector3M eci = chain.RotateSensorToEci(Vector(1.0, 0.0, 0.0));
  EXPECT_NEAR(eci.x, 0.0, 1.0e-9);
  EXPECT_NEAR(eci.y, -1.0, 1.0e-9);
  EXPECT_NEAR(eci.z, 0.0, 1.0e-9);
}

TEST(SbirsBoresightChainTest, MisalignmentMakesChainNonIdentity) {
  oneq::coordinate::EulerAnglesDeg misalignment;
  misalignment.pitch_deg = 3.0;
  EXPECT_FALSE(sbirs_sensor::pipeline::SbirsBoresightChain(
                   sbirs_sensor::session::SbirsEulerAnglesDeg{},
                   oneq::coordinate::EulerAnglesDeg{}, misalignment)
                   .IsIdentity());
}

TEST(SbirsBoresightChainTest, MisalignmentRoundTripPreservesDirection) {
  // R·Rᵀ = I：带失准的链 ECI->Sensor 往返保持方向与模长。
  sbirs_sensor::session::SbirsEulerAnglesDeg attitude;
  attitude.yaw_deg = 20.0;
  attitude.pitch_deg = -6.0;
  oneq::coordinate::EulerAnglesDeg mount;
  mount.roll_deg = 9.0;
  oneq::coordinate::EulerAnglesDeg misalignment;
  misalignment.yaw_deg = -15.0;
  misalignment.pitch_deg = 2.0;
  const sbirs_sensor::pipeline::SbirsBoresightChain chain(attitude, mount, misalignment);
  const sbirs_sensor::session::SbirsVector3M probe = Vector(0.4, 0.5, 0.3);
  const sbirs_sensor::session::SbirsVector3M round_trip =
      chain.RotateEciToSensor(chain.RotateSensorToEci(probe));
  EXPECT_NEAR(round_trip.x, probe.x, 1.0e-9);
  EXPECT_NEAR(round_trip.y, probe.y, 1.0e-9);
  EXPECT_NEAR(round_trip.z, probe.z, 1.0e-9);
}

}  // namespace
