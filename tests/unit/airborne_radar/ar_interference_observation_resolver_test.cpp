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
      DefaultFrame(), /*perturbation_seed=*/42U, &forward));
  ASSERT_EQ(forward.size(), 1U);
  EXPECT_EQ(forward.front().observation_id, 1U);
  EXPECT_GT(forward.front().jammer_to_noise_db, 0.0);

  std::reverse(scene.emissions.begin(), scene.emissions.end());
  auto reversed_links = links;
  std::reverse(reversed_links.begin(), reversed_links.end());
  std::vector<session::ArInterferenceObservation> reverse;
  ASSERT_TRUE(TryResolveArInterferenceObservations(
      scene, receiver, oneq::electromagnetics::RfEmissionIdentity{1U, 3U, 4U}, reversed_links, 1.0,
      0.0, DefaultFrame(), /*perturbation_seed=*/42U, &reverse));
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
      DefaultFrame(), /*perturbation_seed=*/42U, &observations));
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
      {MakeLink(scene.emissions.front(), 100.0)}, 1.0, 0.0, DefaultFrame(),
      /*perturbation_seed=*/42U, &observations));
  ASSERT_EQ(observations.size(), 1U);
  EXPECT_EQ(observations.front().deception_class, session::DeceptionClass::kNone);
  EXPECT_EQ(observations.front().coherent_emission_count, 1U);
}

// RGPO/VGPO 可观测特征：kPulseTrain 发射的载频偏移与首脉冲时延应被填充。
// - estimated_carrier_offset_hz = 发射载频 − 接收机调谐载频；
// - estimated_first_pulse_delay_s = (first_pulse_time − window_start) − range/c。
TEST(ArInterferenceObservationResolverTest, PulseTrainPopulatesCarrierOffsetAndFirstPulseDelay) {
  constexpr double kLightSpeed = 299792458.0;
  oneq::electromagnetics::RfSceneFrame scene;
  scene.world_cycle_index = 1U;
  scene.window_start_time_s = 10.0;
  scene.window_duration_s = 1.0;
  // 距离 1000m（发射在 x=1000）。载频 10.5 GHz（相对 10 GHz 本振偏移 +0.5 GHz）。
  // 首脉冲窗口相对时间设为 5e-6 s（相对窗口边界人为延迟）。
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity = oneq::electromagnetics::RfEmissionIdentity{10U, 20U, 1U};
  emission.position_ecef_m.x_m = 1000.0;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      /*first_pulse_time_s=*/10.0 + 5.0e-6, 10.5e9, 20.0e6, 10.0, 1.0e-6, 1.0e-3, 5U, 0.0, 0U, 0U,
      &emission.waveform));
  scene.emissions = {emission};
  oneq::electromagnetics::RfSceneReceiverState receiver;
  receiver.platform_id = 1U;
  receiver.equipment_id = 2U;
  receiver.antenna.half_power_beamwidth_deg = 4.0;
  receiver.center_frequency_hz = 10.0e9;  // 本振调谐载频

  std::vector<session::ArInterferenceObservation> observations;
  ASSERT_TRUE(TryResolveArInterferenceObservations(
      scene, receiver, oneq::electromagnetics::RfEmissionIdentity{1U, 3U, 4U},
      {MakeLink(scene.emissions.front(), 100.0)}, 1.0, 0.0, DefaultFrame(),
      /*perturbation_seed=*/42U, &observations));
  ASSERT_EQ(observations.size(), 1U);
  const auto& obs = observations.front();
  // 载频偏移 = 10.5e9 − 10.0e9 = 0.5e9 Hz。
  EXPECT_NEAR(obs.estimated_carrier_offset_hz, 0.5e9, 1.0e3);
  // 首脉冲时延 = 5e-6 − (1000 / c) ≈ 5e-6 − 3.3356e-6 ≈ 1.664e-6 s。
  const double expected_delay_s = 5.0e-6 - 1000.0 / kLightSpeed;
  EXPECT_NEAR(obs.estimated_first_pulse_delay_s, expected_delay_s, 1.0e-9);
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
      {MakeLink(scene.emissions.front(), 10.0)}, 1.0, 0.0, DefaultFrame(),
      /*perturbation_seed=*/42U, &low));
  ASSERT_TRUE(TryResolveArInterferenceObservations(
      scene, receiver, oneq::electromagnetics::RfEmissionIdentity{1U, 3U, 4U},
      {MakeLink(scene.emissions.front(), 100.0)}, 1.0, 0.0, DefaultFrame(),
      /*perturbation_seed=*/42U, &high));
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
      {MakeLink(scene.emissions.front(), 100.0)}, 1.0, 0.0, frame, /*perturbation_seed=*/42U,
      &observations));
  ASSERT_EQ(observations.size(), 1U);
  const auto& obs = observations.front();
  // ECEF 切平面方位：delta=(0,1000,0)，atan2(1000,0)=90 度（这是 ECEF 切平面伪角度，
  // 与物理视线无关，正是旧代码的输出）。
  EXPECT_NEAR(obs.estimated_bearing_azimuth_deg, 90.0, 1.0);
  // 局部系方位：ENU az=0（正东），yaw=30 后局部系方位应约为 -30 度。
  EXPECT_NEAR(obs.estimated_bearing_azimuth_local_deg, -30.0, 1.5);
  // 两者必须显著不同——这是跨系 bug 的直接判据。
  EXPECT_NE(obs.estimated_bearing_azimuth_local_deg, obs.estimated_bearing_azimuth_deg);
  // slant_range 字段已去真值化（叠加测量噪声），不再是精确 1000m；断言落在真值附近
  // 合理区间内（测量噪声 std ≈ 20m，5σ 容差覆盖单次确定性扰动）。
  EXPECT_NEAR(obs.estimated_slant_range_m, 1000.0, 120.0);
}

// 去真值化回归：斜距/径向速度必须不再是精确仿真真值（contract.md:348），且同种子同输入
// 下可复现（replay 稳定）。此前实现直接写真值几何，使真实输出通道泄漏精确真值。
TEST(ArInterferenceObservationResolverTest, RangeAndRangeRateArePerturbedFromTruth) {
  oneq::electromagnetics::RfSceneFrame scene;
  scene.world_cycle_index = 1U;
  scene.window_start_time_s = 10.0;
  scene.window_duration_s = 1.0;
  // 发射体在接收机正东 1000m，沿视线以 100 m/s 远离（径向速度 +100 m/s）。
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity = oneq::electromagnetics::RfEmissionIdentity{10U, 20U, 1U};
  emission.position_ecef_m.x_m = 1000.0;
  emission.position_ecef_m.y_m = 0.0;
  emission.velocity_ecef_mps.x_mps = 100.0;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(10.0, 1.0, 10.0e9, 20.0e6, 10.0,
                                                               &emission.waveform));
  scene.emissions = {emission};
  oneq::electromagnetics::RfSceneReceiverState receiver;
  receiver.platform_id = 1U;
  receiver.equipment_id = 2U;
  receiver.antenna.half_power_beamwidth_deg = 4.0;

  const auto resolve = [&](std::uint32_t seed) {
    std::vector<session::ArInterferenceObservation> out;
    EXPECT_TRUE(TryResolveArInterferenceObservations(
        scene, receiver, oneq::electromagnetics::RfEmissionIdentity{1U, 3U, 4U},
        {MakeLink(scene.emissions.front(), 100.0)}, 1.0, 0.0, DefaultFrame(), seed, &out));
    return out;
  };

  const auto first = resolve(99U);
  const auto second = resolve(99U);
  ASSERT_EQ(first.size(), 1U);
  ASSERT_EQ(second.size(), 1U);
  // 真值：斜距 1000m，径向速度 +100 m/s。去真值化后两者都不再精确等于真值。
  EXPECT_NE(first.front().estimated_slant_range_m, 1000.0);
  EXPECT_NE(first.front().estimated_range_rate_mps, 100.0);
  // 同种子同输入 → 完全一致（确定性，replay 可复现）。
  EXPECT_DOUBLE_EQ(first.front().estimated_slant_range_m,
                   second.front().estimated_slant_range_m);
  EXPECT_DOUBLE_EQ(first.front().estimated_range_rate_mps,
                   second.front().estimated_range_rate_mps);
  // 不同种子 → 扰动不同（跨周期/跨设备互不相关）。
  const auto other_seed = resolve(200U);
  ASSERT_EQ(other_seed.size(), 1U);
  EXPECT_NE(first.front().estimated_slant_range_m,
            other_seed.front().estimated_slant_range_m);
}

}  // namespace
}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar
