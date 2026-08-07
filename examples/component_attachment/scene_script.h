/**
 * @file scene_script.h
 * @brief 自定义实体-组件示例：世界模型目标真值脚本。
 *
 * 消费方世界模型：场景文件（scenes/*.json，见 scene_data.h）的目标脚本
 * （ScriptedTarget）→ ECEF 运动学状态（TargetEcefState，四通道共享同一
 * 物理目标）→ 各传感器周期输入真值（AR/ESR/EOS/SBIRS/SAR）+ 欧拉推进。
 * 与 behavior_layer 的世界模型脚本同源（同构转换函数，场景数据各自独立）。
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
#include "scene_data.h"

namespace component_attachment {
namespace demo {

/// 目标 ECEF 运动学状态（四通道共享同一物理目标；id/外观/RCS/辐射源频率
/// 随真值流转，转换函数不再按数组下标回查脚本）。
struct TargetEcefState {
  std::uint32_t id{0U};               /**< 外部目标标识 */
  oneq::coordinate::EcefPositionM position{};  /**< ECEF 位置 */
  oneq::coordinate::EcefVelocityMps velocity{}; /**< ECEF 速度 */
  float rcs{0.0f};                    /**< 雷达截面积（m²） */
  float temperature_k{0.0f};          /**< 等效温度（EOS/SBIRS 外观） */
  float projected_area_m2{0.0f};      /**< 等效投影面积（EOS/SBIRS 外观，m²） */
  double emitter_center_frequency_hz{0.0}; /**< ESR 辐射源中心频率（Hz；≤0 = 不配辐射源） */
};

/// 目标脚本 → ECEF 状态（方位/距离/高度经库内 ENU 偏移函数投影到 ECEF，
/// 速度经 ENU 速度函数投影）。脚本为场景文件合法输入，投影调用不会失败
/// （失败时位置留默认零向量）。
std::vector<TargetEcefState> MakeTargetStates(
    const std::vector<ScriptedTarget>& script,
    const oneq::coordinate::LlaPositionDegM& platform_origin);

/// AR 世界目标事实（ECEF 运动学 + RCS）。
std::vector<airborne_radar::session::ArTargetInput> MakeArTargetInputs(
    const std::vector<TargetEcefState>& states);

/// ESR 辐射源真值：与目标一一对应（脉冲列波形，供统计检测门限在
/// pfa=1e-6 下以多脉冲积分过检）。波形共享参数见 esr 块，中心频率在
/// 目标条目内（≤0 的目标不配辐射源）。
std::vector<oneq::electromagnetics::RfSceneEmission> MakeEmitterTruths(
    const std::vector<TargetEcefState>& states, const EsrEmitterParams& esr,
    double window_start_time_s);

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
