/**
 * @file scene_script.h
 * @brief 自定义实体-组件示例：世界模型目标真值脚本。
 *
 * 消费方世界模型：场景文件（scenes/*.json，见 scene_data.h）的目标脚本
 * （ScriptedTarget）→ ECEF 运动学状态（TargetEcefState，四通道共享同一
 * 物理目标）→ 各传感器周期输入真值（AR/ESR/EOS/SBIRS/SAR）+ 欧拉推进。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_SCENE_SCRIPT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_SCENE_SCRIPT_H_

#include <cstdint>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/electromagnetics/RfScene.h"
#include "1q/remote_identification_radar/session/RirSceneTypes.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"
#include "scene_data.h"

namespace component_attachment {
namespace demo {

/// 目标 ECEF 运动学状态（四通道共享同一物理目标；id/外观/RCS/辐射源频率
/// 随真值流转，转换函数不再按数组下标回查脚本）。
struct TargetEcefState {
  std::uint32_t id{0U};               /**< 外部目标标识 */
  std::string type{"air"};            /**< 实体类型（air / ground；可视化落盘用） */
  oneq::coordinate::EcefPositionM position{};  /**< ECEF 位置 */
  oneq::coordinate::EcefVelocityMps velocity{}; /**< ECEF 速度 */
  float rcs{0.0f};                    /**< 雷达截面积（m²） */
  float temperature_k{0.0f};          /**< 等效温度（EOS 外观） */
  float projected_area_m2{0.0f};      /**< 等效投影面积（EOS 外观，m²） */
  double radiant_intensity_w_per_sr{0.0}; /**< 目标辐射强度（SBIRS 外观，W/sr） */
  double emitter_center_frequency_hz{0.0}; /**< ESR 辐射源中心频率（Hz；≤0 = 不配辐射源） */
  bool has_rir_features{false};            /**< 是否携带 RIR 识别特征真值（从脚本流转） */
  bool has_rir_polarization{false};        /**< 是否提供极化通道真值（决定铺极化样本） */
  bool has_rir_pol_cross{false};           /**< 是否显式给出交叉极化 RCS */
  bool has_rir_pol_phase{false};           /**< 是否显式给出 HH–VV 相位 */
  double rir_rcs_dbsm{0.0};                /**< 视角 RCS 网格值（dBsm） */
  double rir_pol_ch1_dbsm{0.0};            /**< 极化通道 1 RCS（dBsm） */
  double rir_pol_ch2_dbsm{0.0};            /**< 极化通道 2 RCS（dBsm） */
  double rir_pol_cross_dbsm{0.0};          /**< 交叉极化 HV=VH RCS（dBsm） */
  double rir_pol_phase_vv_deg{0.0};        /**< VV 相对 HH 相位（deg） */
  std::string rir_truth_model{};           /**< 真值型号名（人读 + 识别准确率统计） */
  std::vector<RirScattererScript> rir_scatterers{}; /**< 距离向散射中心脚本 */
  std::vector<TargetManeuver> maneuvers{}; /**< 变速机动表（从脚本拷贝，推进时按周期查表） */
};

/// 目标脚本 → ECEF 状态（方位/距离/高度经库内 ENU 偏移函数投影到 ECEF，
/// 速度经 ENU 速度函数投影）。脚本为场景文件合法输入，投影调用不会失败
/// （失败时位置留默认零向量）。
std::vector<TargetEcefState> MakeTargetStates(
    const std::vector<ScriptedTarget>& script,
    const oneq::coordinate::LlaPositionDegM& platform_origin);

/// ESR 辐射源真值：与目标一一对应（脉冲列波形，供统计检测门限在
/// pfa=1e-6 下以多脉冲积分过检）。波形共享参数见 esr 块，中心频率在
/// 目标条目内（≤0 的目标不配辐射源）。
std::vector<oneq::electromagnetics::RfSceneEmission> MakeEmitterTruths(
    const std::vector<TargetEcefState>& states, const EsrEmitterParams& esr,
    double window_start_time_s);

/// SBIRS 红外目标真值：同一物理目标（红外签名以辐射强度 W/sr 提供，与 EOS 温度型外观不同源）。
std::vector<sbirs_sensor::session::SbirsSceneTarget> MakeSbirsTargetInputs(
    const std::vector<TargetEcefState>& states);

/// SAR 点目标真值：同一物理目标（LLA 位置 + RCS，m² → dBsm）。
std::vector<sar::session::SarPointTarget> MakeSarPointTargets(
    const std::vector<TargetEcefState>& states);

/// RIR 场景目标真值：同一物理目标经公共 TryMakeEnuSceneState 投影到站点局部 ENU；
/// 携带特征脚本的目标铺均匀视角 RCS 网格 + 双通道极化
/// 样本 + 散射器（仿集成测试配方，识别库模板匹配源），无特征脚本的目标只供
/// 标量 RCS（探测链可用、无识别结论）。
std::vector<remote_identification_radar::session::RirSceneTarget> MakeRirSceneTargets(
    const std::vector<TargetEcefState>& states,
    const oneq::coordinate::LlaPositionDegM& site_origin);

/// 目标 ECEF 欧拉推进（消费方世界模型）。每周期先
/// 应用变速机动（maneuvers 中 start_cycle == cycle 的条目生效，分段匀速；
/// 机动速度为局部 ENU，经 platform_origin 投影回 ECEF，与初始速度投影同源），
/// 再按当前速度推进。
void AdvanceTargetStates(std::vector<TargetEcefState>& states, std::uint32_t cycle,
                         double dt_s,
                         const oneq::coordinate::LlaPositionDegM& platform_origin);

}  // namespace demo
}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_SCENE_SCRIPT_H_
