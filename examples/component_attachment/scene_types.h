/**
 * @file scene_types.h
 * @brief 自定义实体-组件示例：演示场景的共享场景状态。
 *
 * DemoSceneState 继承 core 的 SceneState（周期号/时间）并扩展四通道世界
 * 真值（AR 目标 / ESR 辐射源 / EOS 光学目标 / SBIRS 红外目标 + 天基平台
 * 位置）——演示"共享上下文继承扩展"模式（消费方继承 SceneState 扩展自有
 * 字段）。消费方（demo）每周期更新字段，组件 Step 经
 * world.scene_state() 类型化访问。SAR 点目标真值见 sar_point_targets。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_SCENE_TYPES_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_SCENE_TYPES_H_

#include <vector>

#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "1q/electromagnetics/RfScene.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"
#include "core/world.h"

namespace component_attachment {

/** @brief 演示场景共享状态：四传感器世界真值 + 天基平台（消费方脚本注入）。 */
struct DemoSceneState : SceneState {
  std::vector<airborne_radar::session::ArTargetInput> ar_targets{}; /**< AR 世界目标事实 */
  std::vector<oneq::electromagnetics::RfSceneEmission> emitters{}; /**< ESR 辐射源真值 */
  std::vector<electro_optical_sensor::session::EosExternalTargetInput> optical_targets{}; /**< EOS 光学目标 */
  std::vector<sbirs_sensor::session::SbirsSceneTarget> sbirs_targets{}; /**< SBIRS 红外目标真值 */
  sbirs_sensor::session::SbirsVector3M sbirs_satellite_position_ecef_m{}; /**< 天基平台（卫星）ECEF 位置 */
  sbirs_sensor::session::SbirsVector3M sbirs_satellite_velocity_ecef_m_per_s{}; /**< 天基平台（卫星）ECEF 速度（必填；演示合成静止卫星，缺省零向量合法） */
  sbirs_sensor::session::SbirsEulerAnglesDeg sbirs_satellite_attitude_eci_body_deg{}; /**< 天基平台（卫星）姿态（Z-Y-X，Body->ECI；必填，缺省零欧拉 = 体轴对齐 ECI） */
  // 缺省与 SceneData 一致（2024-01-01 00:00 UTC）；0 = 未提供会被库校验拒绝。
  double sbirs_utc_julian_day{2460310.5}; /**< 天基通道 UTC 儒略日（JD_UTC；SBIRS ECI 输出参考系必需） */
  std::vector<sar::session::SarPointTarget> sar_point_targets{}; /**< SAR 点目标真值（LLA + RCS） */
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_SCENE_TYPES_H_
