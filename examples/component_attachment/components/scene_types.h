/**
 * @file scene_types.h
 * @brief 自定义实体-组件示例：演示场景的共享场景状态。
 *
 * DemoSceneState 继承 core 的 SceneState（周期号/时间）并扩展四通道世界
 * 真值（AR 目标 / ESR 辐射源 / EOS 光学目标 / SBIRS 红外目标 + 天基平台
 * 位置）——演示"共享上下文继承扩展"模式（对应 EnTT registry.ctx() 的
 * 消费方扩展）。消费方（demo）每周期更新字段，组件 Step 经
 * world.scene_state() 类型化访问。SAR 点目标真值见 sar_point_targets。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SCENE_TYPES_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SCENE_TYPES_H_

#include <vector>

#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "1q/electromagnetics/RfScene.h"
#include "1q/sar/session/SarCycleInput.h"
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
  std::vector<sar::session::SarPointTarget> sar_point_targets{}; /**< SAR 点目标真值（LLA + RCS） */
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SCENE_TYPES_H_
