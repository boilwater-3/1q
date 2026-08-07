/**
 * @file scene_script.h
 * @brief 自定义实体-组件示例：世界模型目标真值脚本。
 *
 * 消费方世界模型：目标脚本（ScriptedTarget，见 scene_script.cpp 内部）→
 * ECEF 运动学状态（TargetEcefState，四通道共享同一物理目标）→ 各传感器
 * 周期输入真值（AR/ESR/EOS/SBIRS/SAR）+ 欧拉推进。与 behavior_layer 的
 * 世界模型脚本同源。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_SCENE_SCRIPT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_SCENE_SCRIPT_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/coordinate/types.h"
#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "1q/electromagnetics/RfScene.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"

namespace component_attachment {
namespace demo {

/// 目标 ECEF 运动学状态（三通道共享同一物理目标）。
struct TargetEcefState {
  oneq::coordinate::EcefPositionM position{};
  oneq::coordinate::EcefVelocityMps velocity{};
  float rcs{0.0f};
};

/// 目标脚本 → ECEF 状态（方位/距离/高度经库内 ENU 偏移函数投影到 ECEF，速度经
/// ENU 速度函数投影；高度 = 巡航高度，目标恒在空中且与平台巡航同高，不随
/// 平台起飞段高度变化）。脚本为编译期合法常量，投影调用不会失败。
std::vector<TargetEcefState> MakeTargetStates(
    const oneq::coordinate::LlaPositionDegM& platform_origin);

/// AR 世界目标事实（ECEF 运动学 + RCS）。
std::vector<airborne_radar::session::ArTargetInput> MakeArTargetInputs(
    const std::vector<TargetEcefState>& states);

/// ESR 辐射源真值：与 AR 目标同一物理目标（脉冲列波形，供统计检测门限
/// 在 pfa=1e-6 下以多脉冲积分过检）。两辐射源中心频率互异（9.5/10.0 GHz），
/// 保证 ESR 分选聚簇能稳定分离出 2 条假设航迹。
std::vector<oneq::electromagnetics::RfSceneEmission> MakeEmitterTruths(
    const std::vector<TargetEcefState>& states, double window_start_time_s);

/// EOS 光学目标真值：同一物理目标（外观参数仿 electro_optical 示例）。
std::vector<electro_optical_sensor::session::EosExternalTargetInput> MakeOpticalTargets(
    const std::vector<TargetEcefState>& states);

/// SBIRS 红外目标真值：同一物理目标（红外外观参数与 EOS 同源）。
std::vector<sbirs_sensor::session::SbirsSceneTarget> MakeSbirsTargetInputs(
    const std::vector<TargetEcefState>& states);

/// SAR 点目标真值：同一物理目标（LLA 位置 + RCS，m² → dBsm）。
std::vector<sar::session::SarPointTarget> MakeSarPointTargets(
    const std::vector<TargetEcefState>& states);

/// 目标 ECEF 欧拉推进（消费方世界模型，与 behavior_layer 一致）。
void AdvanceTargetStates(std::vector<TargetEcefState>& states, double dt_s);

}  // namespace demo
}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_SCENE_SCRIPT_H_
