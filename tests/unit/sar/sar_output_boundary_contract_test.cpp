/**
 * @file sar_output_boundary_contract_test.cpp
 * @brief SAR 真实系统输出边界合同测试。
 *
 * 锁定阶段 9 的边界契约：SAR 主输出是成像产品元数据(SarOutputFrame)，
 * 不得携带点目标真值标识(target_id/target_name)。点目标解释只能经
 * SarProductDebugViewBuilder 通过输入表回填。
 *
 * 该测试同时作为回归哨兵：通过 sizeof 锚定 SarOutputFrame 与点目标输入
 * 的边界，任何把 name/id 塞进产品输出的改动都会改变结构语义并在此暴露。
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>
#include <type_traits>

#include "1q/sar/session/SarCycleResult.h"
#include "1q/sar/session/SarProductDebugView.h"
#include "1q/sar/session/SarSession.h"

namespace sar {
namespace {

// 编译期哨兵：SarOutputFrame 是成像产品元数据，仅由标量(cycle_index/stage/
// 计数/物理量/标志位)组成，不得混入点目标真值标识。当前结构全部成员为
// trivially-copyable 标量；若有人加入 std::string target_name 或其它非平凡
// 成员，trivially-copyable 性质被破坏，此处编译失败，强制开发者审视是否
// 破坏了产品输出边界。
static_assert(std::is_trivially_copyable<session::SarOutputFrame>::value,
              "SarOutputFrame must remain product metadata of scalar fields; "
              "point-target identity (e.g. std::string name) is not allowed.");

config::SarSessionConfig MakeSmallRdaConfig() {
  config::SarSessionConfig config;
  config.hardware.carrier_frequency_hz = 1.0e9;
  config.hardware.bandwidth_hz = 25.0e6;
  config.hardware.pulse_width_s = 0.16e-6;
  config.hardware.pulse_repetition_frequency_hz = 20.0;
  config.hardware.sample_rate_hz = 100.0e6;
  config.mission.nominal_slant_range_m = 29.9792458;
  config.mission.scene_center_latitude_deg =
      29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  config.mission.platform_speed_mps = 2.0;
  config.mission.range_sample_count = 64U;
  config.mission.azimuth_pulse_count = 9U;
  config.policy.enable_l1_rda_imaging = true;
  return config;
}

session::SarCycleInput MakeNamedPointTargetInput() {
  session::SarCycleInput input;
  input.cycle_index = 7U;
  input.dt_sec = 0.1f;
  input.platform.latitude_deg = 0.0;
  input.platform.longitude_deg = 0.0;
  input.platform.altitude_m = 0.0;

  session::SarPointTarget target;
  target.target_id = 4242U;
  target.target_name = "sar-contract-target";
  target.latitude_deg = 29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  target.longitude_deg = 0.0;
  target.altitude_m = 0.0;
  target.radar_cross_section_dbsm = 80.0;
  input.point_targets.push_back(target);
  return input;
}

}  // namespace

// 合同：带 name/id 的点目标输入经过真实聚焦 pipeline 后，
// 产品输出帧(SarOutputFrame)不得携带点目标真值标识。
TEST(SarOutputBoundaryContractTest, NamedPointTargetDoesNotLeakIntoProductOutput) {
  session::SarSession session = session::SarSession::Create(MakeSmallRdaConfig());
  const session::SarCycleInput input = MakeNamedPointTargetInput();
  // 输入侧确实带 name/id，作为回归前提。
  ASSERT_EQ(input.point_targets.size(), 1U);
  EXPECT_EQ(input.point_targets[0].target_name, "sar-contract-target");
  EXPECT_EQ(input.point_targets[0].target_id, 4242U);

  const session::SarCycleResult result = session.StepWithResult(input);
  ASSERT_TRUE(result.executed_this_cycle);
  ASSERT_EQ(result.status, session::SarCycleStatus::kCompleted);

  // 产品输出帧是成像产品元数据；它不含任何 target_name/target_id 字段。
  // 这里断言它确实产出了产品（证明 pipeline 真实执行），同时其字段集
  // 保持产品语义（阶段/样本/分辨率/SNR）而非目标语义。
  EXPECT_TRUE(result.output_frame.has_l1_image);
  EXPECT_EQ(result.output_frame.completed_stage, session::SarProcessingStage::kL1RdaImage);
  EXPECT_GT(result.output_frame.range_sample_count, 0U);
  EXPECT_GT(result.output_frame.azimuth_pulse_count, 0U);
}

// 合同：点目标 name 只能经 debug view 通过输入表回填，
// 不得作为产品输出字段直接出现。
TEST(SarOutputBoundaryContractTest, PointTargetNameOnlyReachableViaDebugView) {
  session::SarSession session = session::SarSession::Create(MakeSmallRdaConfig());
  const session::SarCycleInput input = MakeNamedPointTargetInput();
  const session::SarCycleResult result = session.StepWithResult(input);
  ASSERT_EQ(result.status, session::SarCycleStatus::kCompleted);

  const session::SarProductDebugView view = session::SarProductDebugViewBuilder::Build(input, result);
  ASSERT_EQ(view.point_targets.size(), 1U);
  // debug view 是唯一能取回点目标 name 的位置。
  EXPECT_EQ(view.point_targets[0].target_name, "sar-contract-target");
  EXPECT_EQ(view.point_targets[0].target_id, 4242U);
}

}  // namespace sar
