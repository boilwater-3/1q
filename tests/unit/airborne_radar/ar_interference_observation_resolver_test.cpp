#include <gtest/gtest.h>

#include <algorithm>

#include "airborne_radar/signal/detection/ArInterferenceObservationResolver.h"

namespace airborne_radar {
namespace signal {
namespace detection {
namespace {

// 默认平台坐标系：ECEF 原点 + 零姿态（ENU）。既有用例的接收机位于 ECEF 原点，
// 故局部系方位与既有 ECEF 切平面方位在小偏移量下数值一致，行为保持不变。
oneq::coordinate::LocalFrameReference DefaultFrame() {
  oneq::coordinate::LocalFrameReference frame;
  frame.origin_lla = oneq::coordinate::LlaPositionDegM{0.0, 0.0, 0.0};
  frame.frame_attitude_deg = oneq::coordinate::EulerAnglesDeg{0.0, 0.0, 0.0};
  return frame;
}

oneq::electromagnetics::RfSceneEmission MakeJammer(std::uint64_t emission_id, double y_m,
                                                   double power_w) {
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity = oneq::electromagnetics::RfEmissionIdentity{10U, 20U, emission_id};
  emission.position_ecef_m.x_m = 1000.0;
  emission.position_ecef_m.y_m = y_m;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(10.0, 1.0, 10.0e9, 20.0e6, power_w,
                                                               &emission.waveform));
  return emission;
}

// 构造一个同方位、相近 y 偏移的脉冲列欺骗发射，用于触发假目标方位聚类。
oneq::electromagnetics::RfSceneEmission MakePulseTrainJammer(std::uint64_t emission_id,
                                                              double y_m, double power_w) {
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity = oneq::electromagnetics::RfEmissionIdentity{10U, 20U, emission_id};
  emission.position_ecef_m.x_m = 1000.0;
  emission.position_ecef_m.y_m = y_m;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      10.0, 10.0e9, 20.0e6, power_w, 1.0e-6, 1.0e-3, 5U, 0.0, 0U, 0U, &emission.waveform));
  return emission;
}

oneq::electromagnetics::RfIncidentLinkResult MakeLink(
    const oneq::electromagnetics::RfSceneEmission& emission, double power_w) {
  oneq::electromagnetics::RfIncidentLinkResult link;
  link.identity = emission.identity;
  link.received_power_w = power_w;
  return link;
}

TEST(ArInterferenceObservationResolverTest, GatesByJOverNAndIsOrderIndependent) {
  oneq::electromagnetics::RfSceneFrame scene;
  scene.world_cycle_index = 1U;
  scene.window_start_time_s = 10.0;
  scene.window_duration_s = 1.0;
  scene.emissions = {MakeJammer(2U, 100.0, 10.0), MakeJammer(1U, -100.0, 10.0)};
  oneq::electromagnetics::RfSceneReceiverState receiver;
  receiver.platform_id = 1U;
  receiver.equipment_id = 2U;
  receiver.antenna.half_power_beamwidth_deg = 4.0;
  const std::vector<oneq::electromagnetics::RfIncidentLinkResult> links = {
      MakeLink(scene.emissions[0], 100.0), MakeLink(scene.emissions[1], 0.01)};

  std::vector<session::ArInterferenceObservation> forward;
  ASSERT_TRUE(TryResolveArInterferenceObservations(
      scene, receiver, oneq::electromagnetics::RfEmissionIdentity{1U, 3U, 4U}, links, 1.0, 0.0,
      DefaultFrame(), &forward));
  ASSERT_EQ(forward.size(), 1U);
  EXPECT_EQ(forward.front().observation_id, 1U);
  EXPECT_GT(forward.front().jammer_to_noise_db, 0.0);

  std::reverse(scene.emissions.begin(), scene.emissions.end());
  auto reversed_links = links;
  std::reverse(reversed_links.begin(), reversed_links.end());
  std::vector<session::ArInterferenceObservation> reverse;
  ASSERT_TRUE(TryResolveArInterferenceObservations(
      scene, receiver, oneq::electromagnetics::RfEmissionIdentity{1U, 3U, 4U}, reversed_links, 1.0,
      0.0, DefaultFrame(), &reverse));
  ASSERT_EQ(reverse.size(), forward.size());
  EXPECT_DOUBLE_EQ(reverse.front().estimated_bearing_azimuth_deg,
                   forward.front().estimated_bearing_azimuth_deg);
  EXPECT_EQ(reverse.front().observation_id, forward.front().observation_id);
}

TEST(ArInterferenceObservationResolverTest, CoherentPulseTrainEmissionsTaggedAsFalseTarget) {
  // 两个同方位、相近 y 偏移的 kPulseTrain 发射落在同一波束宽度内，应被聚类标记为疑似假目标。
  oneq::electromagnetics::RfSceneFrame scene;
  scene.world_cycle_index = 1U;
  scene.window_start_time_s = 10.0;
  scene.window_duration_s = 1.0;
  // y 偏移 100.0 与 105.0：在 1000m 距离上方位差极小，远小于 4 度波束宽度。
  scene.emissions = {MakePulseTrainJammer(1U, 100.0, 10.0),
                     MakePulseTrainJammer(2U, 105.0, 10.0)};
  oneq::electromagnetics::RfSceneReceiverState receiver;
  receiver.platform_id = 1U;
  receiver.equipment_id = 2U;
  receiver.antenna.half_power_beamwidth_deg = 4.0;
  const std::vector<oneq::electromagnetics::RfIncidentLinkResult> links = {
      MakeLink(scene.emissions[0], 100.0), MakeLink(scene.emissions[1], 100.0)};

  std::vector<session::ArInterferenceObservation> observations;
  ASSERT_TRUE(TryResolveArInterferenceObservations(
      scene, receiver, oneq::electromagnetics::RfEmissionIdentity{1U, 3U, 4U}, links, 1.0, 0.0,
      DefaultFrame(), &observations));
  ASSERT_EQ(observations.size(), 2U);
  // 两条观测应均被标记为疑似假目标，且同方向计数 ≥ 2（含自身）。
  for (const auto& obs : observations) {
    EXPECT_EQ(obs.deception_class, session::DeceptionClass::kLikelyFalseTarget);
    EXPECT_GE(obs.coherent_emission_count, 2U);
  }
}

TEST(ArInterferenceObservationResolverTest, IsolatedPulseTrainNotTaggedAsFalseTarget) {
  // 单条 kPulseTrain 发射无同方向邻居，不应被标记为假目标。
  oneq::electromagnetics::RfSceneFrame scene;
  scene.world_cycle_index = 1U;
  scene.window_start_time_s = 10.0;
  scene.window_duration_s = 1.0;
  scene.emissions = {MakePulseTrainJammer(1U, 100.0, 10.0)};
  oneq::electromagnetics::RfSceneReceiverState receiver;
  receiver.platform_id = 1U;
  receiver.equipment_id = 2U;
  receiver.antenna.half_power_beamwidth_deg = 4.0;

  std::vector<session::ArInterferenceObservation> observations;
  ASSERT_TRUE(TryResolveArInterferenceObservations(
      scene, receiver, oneq::electromagnetics::RfEmissionIdentity{1U, 3U, 4U},
      {MakeLink(scene.emissions.front(), 100.0)}, 1.0, 0.0, DefaultFrame(), &observations));
  ASSERT_EQ(observations.size(), 1U);
  EXPECT_EQ(observations.front().deception_class, session::DeceptionClass::kNone);
  EXPECT_EQ(observations.front().coherent_emission_count, 1U);
}

TEST(ArInterferenceObservationResolverTest, UncertaintyDecreasesWithJOverN) {
  oneq::electromagnetics::RfSceneFrame scene;
  scene.world_cycle_index = 1U;
  scene.window_start_time_s = 10.0;
  scene.window_duration_s = 1.0;
  scene.emissions.push_back(MakeJammer(1U, 0.0, 10.0));
  oneq::electromagnetics::RfSceneReceiverState receiver;
  receiver.platform_id = 1U;
  receiver.equipment_id = 2U;
  receiver.antenna.half_power_beamwidth_deg = 4.0;

  std::vector<session::ArInterferenceObservation> low;
  std::vector<session::ArInterferenceObservation> high;
  ASSERT_TRUE(TryResolveArInterferenceObservations(
      scene, receiver, oneq::electromagnetics::RfEmissionIdentity{1U, 3U, 4U},
      {MakeLink(scene.emissions.front(), 10.0)}, 1.0, 0.0, DefaultFrame(), &low));
  ASSERT_TRUE(TryResolveArInterferenceObservations(
      scene, receiver, oneq::electromagnetics::RfEmissionIdentity{1U, 3U, 4U},
      {MakeLink(scene.emissions.front(), 100.0)}, 1.0, 0.0, DefaultFrame(), &high));
  ASSERT_EQ(low.size(), 1U);
  ASSERT_EQ(high.size(), 1U);
  EXPECT_LT(high.front().bearing_standard_deviation_deg,
            low.front().bearing_standard_deviation_deg);
  EXPECT_LT(high.front().frequency_standard_deviation_hz,
            low.front().frequency_standard_deviation_hz);
}

// 核心回归：平台不在 ECEF 原点、姿态非零时，雷达局部系方位必须与 ECEF 切平面方位不同，
// 且局部系方位应与该视线在雷达局部系下的期望 look angle 一致。此前实现直接输出 ECEF
// 方位，与目标 look angle 跨系比较，导致确定性漏标/误标。
//
// 注意：必须用物理合理的几何（接收机与 frame origin 同处地表，发射体在真实视线方向上），
// 而非既有 fixture 的 ECEF 原点（地心深处）构造——后者使 ENU 变换退化为正上方，方位
// 未定义，无法暴露跨系 bug。
TEST(ArInterferenceObservationResolverTest, LocalFrameBearingDiffersFromEcefWhenAttitudeNonZero) {
  constexpr double kEarthRadiusM = 6378137.0;
  oneq::electromagnetics::RfSceneFrame scene;
  scene.world_cycle_index = 1U;
  scene.window_start_time_s = 10.0;
  scene.window_duration_s = 1.0;
  // 发射体位于接收机正东 1000m：ECEF ≈ (R, 1000, 0)。
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity = oneq::electromagnetics::RfEmissionIdentity{10U, 20U, 1U};
  emission.position_ecef_m.x_m = kEarthRadiusM;
  emission.position_ecef_m.y_m = 1000.0;
  emission.position_ecef_m.z_m = 0.0;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(10.0, 1.0, 10.0e9, 20.0e6, 10.0,
                                                               &emission.waveform));
  scene.emissions = {emission};
  // 接收机位于地表 LLA(0,0,0) => ECEF(R,0,0)，与 frame origin 一致。
  oneq::electromagnetics::RfSceneReceiverState receiver;
  receiver.platform_id = 1U;
  receiver.equipment_id = 2U;
  receiver.antenna.half_power_beamwidth_deg = 4.0;
  receiver.position_ecef_m.x_m = kEarthRadiusM;

  // 平台坐标系：origin LLA(0,0,0)（与接收机一致），偏航 30 度。
  // RotateEnuToLocal 应用 Inverse(Rz(30))，正东视线（ENU az=0）在局部系中方位应约为 -30 度。
  oneq::coordinate::LocalFrameReference frame = DefaultFrame();
  frame.frame_attitude_deg.yaw_deg = 30.0;

  std::vector<session::ArInterferenceObservation> observations;
  ASSERT_TRUE(TryResolveArInterferenceObservations(
      scene, receiver, oneq::electromagnetics::RfEmissionIdentity{1U, 3U, 4U},
      {MakeLink(scene.emissions.front(), 100.0)}, 1.0, 0.0, frame, &observations));
  ASSERT_EQ(observations.size(), 1U);
  const auto& obs = observations.front();
  // ECEF 切平面方位：delta=(0,1000,0)，atan2(1000,0)=90 度（这是 ECEF 切平面伪角度，
  // 与物理视线无关，正是旧代码的输出）。
  EXPECT_NEAR(obs.estimated_bearing_azimuth_deg, 90.0, 1.0);
  // 局部系方位：ENU az=0（正东），yaw=30 后局部系方位应约为 -30 度。
  EXPECT_NEAR(obs.estimated_bearing_azimuth_local_deg, -30.0, 1.5);
  // 两者必须显著不同——这是跨系 bug 的直接判据。
  EXPECT_NE(obs.estimated_bearing_azimuth_local_deg, obs.estimated_bearing_azimuth_deg);
  // slant_range 字段应被填充为约 1000m。
  EXPECT_NEAR(obs.estimated_slant_range_m, 1000.0, 1.0);
}

}  // namespace
}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar
