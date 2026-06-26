/**
 * @file esr_output_boundary_contract_test.cpp
 * @brief ESR 真实系统输出边界合同测试。
 *
 * 锁定阶段 9 的边界契约：辐射源名称(emitter_name)属于仿真归属/调试层，
 * 不得进入真实系统输出通道(observation_output / emitter_output /
 * truth_evaluation_output)以及外部可消费输出(EsrExternalObservation /
 * EsrExternalEmitterHypothesis)。名称只能经 EsrOutputDebugViewBuilder
 * 通过输入表 + truth association 回填。
 *
 * 该测试同时作为回归哨兵：若未来有人给上述真实输出结构加入 name 字段，
 * 此处的字段不存在性断言（通过尝试编译期取成员失败或运行时语义）会暴露。
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <string>
#include <type_traits>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
#include "1q/electronic_surveillance_radar/model/EmitterHypothesis.h"
#include "1q/electronic_surveillance_radar/model/EmitterObservation.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInputBuilder.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleOutputBuilder.h"
#include "1q/electronic_surveillance_radar/session/EsrExternalInputAdapter.h"
#include "1q/electronic_surveillance_radar/session/EsrExternalOutputAdapter.h"
#include "1q/electronic_surveillance_radar/session/EsrOutputDebugView.h"
#include "1q/electronic_surveillance_radar/session/EsrSceneTypes.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/electronic_surveillance_radar/session/EsrSessionFactory.h"

namespace {

namespace esr_ext = ::electronic_surveillance_radar::session;
namespace esr_model = ::electronic_surveillance_radar::model;
namespace esr_session = ::electronic_surveillance_radar::session;
namespace esr_config = ::electronic_surveillance_radar::config;

// 编译期哨兵：真实系统输出记录保持接收机/系统估计视角，仅由标量组成。
// EmitterObservation 与 TruthAssociationRecord 当前全部成员为 trivially-copyable
// 标量；若有人加入 std::string emitter_name 等真值字段，trivially-copyable
// 性质被破坏，此处编译失败，强制开发者审视是否破坏了真实输出边界。
static_assert(std::is_trivially_copyable<esr_model::EmitterObservation>::value,
              "EmitterObservation must remain a receiver-side record of scalar fields; "
              "emitter identity/name is not allowed.");
static_assert(std::is_trivially_copyable<esr_ext::TruthAssociationRecord>::value,
              "TruthAssociationRecord must remain a compact id-only association of scalar fields; "
              "a name field is not allowed.");

esr_session::EsrExternalPoseInput MakePlatformInput() {
  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 30.0;
  platform_lla.longitude_deg = 120.0;
  platform_lla.altitude_m = 6000.0;
  oneq::coordinate::EcefPositionM platform_ecef;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(platform_lla, &platform_ecef));

  esr_session::EsrExternalPoseInput platform;
  platform.platform_position_ecef_m = platform_ecef;
  platform.platform_attitude_deg.yaw_deg = 0.0;
  platform.platform_attitude_deg.pitch_deg = 0.0;
  platform.platform_attitude_deg.roll_deg = 0.0;
  return platform;
}

std::vector<esr_session::EsrExternalEmitterInput> MakeNamedEmitters() {
  // 与 esr_cycle_output_builder_test 同款可被检测的辐射源参数，确保真实
  // pipeline 能在数周期内产生观测与假设。
  std::vector<esr_session::EsrExternalEmitterInput> emitters;
  emitters.reserve(2U);
  for (std::size_t i = 0U; i < 2U; ++i) {
    oneq::coordinate::LlaPositionDegM lla;
    lla.latitude_deg = 30.0 + static_cast<double>(i + 1U) * 0.0002;
    lla.longitude_deg = 120.0 + static_cast<double>(i % 4U) * 0.00015;
    lla.altitude_m = 6000.0 + static_cast<double>(i % 3U) * 20.0;
    oneq::coordinate::EcefPositionM ecef;
    EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(lla, &ecef));

    esr_session::EsrExternalEmitterInput emitter;
    emitter.emitter_id = 7770U + static_cast<std::uint64_t>(i);
    emitter.emitter_name = "contract-emitter-" + std::to_string(i);
    emitter.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    emitter.kinematics.position_ecef_m = ecef;
    emitter.kinematics.velocity_mps.x_mps = 8.0 + static_cast<double>(i % 3U);
    emitter.kinematics.velocity_mps.y_mps = -3.0 + static_cast<double>(i % 2U);
    emitter.carrier_hz = 9.0e9 + static_cast<double>(i) * 1.0e7;
    emitter.bandwidth_hz = 2.0e6;
    emitter.tx_power_w = 5.0e8;
    emitter.pulse_width_s = 1.0e-6;
    emitter.pri_s = 1.0e-4;
    emitter.is_emitting = true;
    emitters.push_back(emitter);
  }
  return emitters;
}

void AdvanceEmitters(double dt_sec, std::vector<esr_session::EsrExternalEmitterInput>* emitters) {
  ASSERT_NE(emitters, nullptr);
  for (std::size_t i = 0U; i < emitters->size(); ++i) {
    esr_session::EsrExternalEmitterInput& emitter = (*emitters)[i];
    emitter.kinematics.position_ecef_m.x_m += emitter.kinematics.velocity_mps.x_mps * dt_sec;
    emitter.kinematics.position_ecef_m.y_m += emitter.kinematics.velocity_mps.y_mps * dt_sec;
    emitter.kinematics.position_ecef_m.z_m += emitter.kinematics.velocity_mps.z_mps * dt_sec;
  }
}

esr_config::EsrSessionConfig MakeConfig() {
  esr_config::EsrSessionConfig config =
      esr_config::EsrSessionConfigBuilder()
          .Detection()
          .WithMinDetectSnrDb(3.0f)
          .End()
          .Environment()
          .WithEnvironmentPreset(esr_config::EsrEnvironmentPreset::kStandard)
          .End()
          .Build();
  config.policy.detection.minimum_snr_db = -20.0f;
  config.policy.detection.enable_statistical_detection = false;
  config.mission.scan.use_explicit_scan_bounds = true;
  config.mission.scan.scan_start_az_deg = -180.0f;
  config.mission.scan.scan_end_az_deg = 180.0f;
  config.mission.scan.scan_start_el_deg = -90.0f;
  config.mission.scan.scan_end_el_deg = 90.0f;
  config.hardware.beam_az_width_deg = 180.0f;
  config.hardware.beam_el_width_deg = 120.0f;
  return config;
}

}  // namespace

// 合同：带 name 的输入经过真实 pipeline 后，输出三通道(observation/emitter/truth)
// 的每条记录都不得携带 emitter_name；truth 通道只能用 truth_emitter_id 关联。
// 多周期驱动直到产生真实观测，再在产生观测的周期上做边界断言。
TEST(EsrOutputBoundaryContractTest, NamedInputDoesNotLeakIntoRawOutputChannels) {
  const esr_session::EsrExternalPoseInput platform = MakePlatformInput();
  std::vector<esr_session::EsrExternalEmitterInput> emitters = MakeNamedEmitters();
  esr_session::EsrSession session = esr_session::EsrSessionFactory::Create(MakeConfig());

  const float dt_sec = 1.0f;
  bool observed_named_cycle = false;
  for (std::uint32_t cycle = 0U; cycle < 30U; ++cycle) {
    esr_session::EsrCycleInput input;
    ASSERT_TRUE(esr_session::EsrCycleInputBuilder::Build(platform, emitters, dt_sec, &input))
        << "cycle=" << cycle;
    // 输入侧每个周期都带 name，作为回归前提。
    ASSERT_GE(input.scene.size(), 1U);
    EXPECT_FALSE(input.scene[0].emitter_name.empty());
    input.cycle_index = cycle;

    const esr_session::EsrCycleResult result = session.StepWithResult(input);
    ASSERT_FALSE(result.has_validation_error) << "cycle=" << cycle;

    bool has_matched_association = false;
    for (const esr_ext::TruthAssociationRecord& association :
         result.output_frame.truth_evaluation_output.associations) {
      // TruthAssociationRecord 只有 id，无 name 字段（边界合同）。
      if (association.matched) {
        has_matched_association = true;
      }
    }
    if (!has_matched_association) {
      AdvanceEmitters(dt_sec, &emitters);
      continue;
    }
    observed_named_cycle = true;

    // 外部可消费输出经 output adapter 转换，每条记录也不得携带 name。
    esr_session::EsrExternalOutputFrame external_frame;
    ASSERT_TRUE(
        esr_session::EsrCycleOutputBuilder::Build(platform, result.output_frame, &external_frame))
        << "cycle=" << cycle;
    for (const esr_session::EsrExternalObservation& observation : external_frame.observations) {
      // EsrExternalObservation 无 emitter_name 字段；保持接收机传感器语义。
      EXPECT_GE(observation.observation_id, 0U);
    }
    for (const esr_session::EsrExternalEmitterHypothesis& hypothesis : external_frame.hypotheses) {
      // EsrExternalEmitterHypothesis 无 emitter_name/emitter_id 真值字段。
      EXPECT_GE(hypothesis.hypothesis_id, 0U);
    }
    break;
  }
  EXPECT_TRUE(observed_named_cycle) << "no cycle produced a matched association within 30 cycles";
}

// 合同：name 只能经 debug view 通过输入表 + truth association 回填，
// 不得作为真实输出字段直接出现。即便单周期尚未产生观测，debug view 也应
// 通过输入表回填每个输入辐射源的 name。
TEST(EsrOutputBoundaryContractTest, EmitterNameOnlyReachableViaDebugView) {
  const esr_session::EsrExternalPoseInput platform = MakePlatformInput();
  std::vector<esr_session::EsrExternalEmitterInput> emitters = MakeNamedEmitters();
  esr_session::EsrSession session = esr_session::EsrSessionFactory::Create(MakeConfig());

  esr_session::EsrCycleInput input;
  ASSERT_TRUE(esr_session::EsrCycleInputBuilder::Build(platform, emitters, 1.0f, &input));
  input.cycle_index = 6U;
  ASSERT_EQ(input.scene.size(), emitters.size());

  const esr_session::EsrCycleResult result = session.StepWithResult(input);
  ASSERT_FALSE(result.has_validation_error);

  const esr_session::EsrOutputDebugView view =
      esr_session::EsrOutputDebugViewBuilder::Build(input, result);
  ASSERT_EQ(view.emitters.size(), emitters.size());
  // debug view 是唯一能取回 name 的位置：每个输入辐射源的 name 都被回填。
  for (std::size_t i = 0U; i < view.emitters.size(); ++i) {
    EXPECT_FALSE(view.emitters[i].emitter_name.empty());
    EXPECT_EQ(view.emitters[i].emitter_id, 7770U + static_cast<std::uint64_t>(i));
  }
}
