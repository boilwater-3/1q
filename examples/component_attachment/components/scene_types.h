/**
 * @file scene_types.h
 * @brief 自定义实体-组件示例：演示场景的共享场景状态。
 *
 * DemoSceneState 继承 core 的 SceneState（周期号/时间）并扩展三通道世界
 * 真值（AR 目标 / ESR 辐射源 / EOS 光学目标）——演示"共享上下文继承
 * 扩展"模式（对应 EnTT registry.ctx() 的消费方扩展）。消费方（demo）每
 * 周期更新字段，组件 Step 经 world.scene_state() 类型化访问。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SCENE_TYPES_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SCENE_TYPES_H_

#include <vector>

#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "1q/electromagnetics/RfScene.h"
#include "core/world.h"

namespace component_attachment {

/** @brief 演示场景共享状态：三传感器世界真值（消费方脚本注入）。 */
struct DemoSceneState : SceneState {
  std::vector<airborne_radar::session::ArTargetInput> ar_targets{}; /**< AR 世界目标事实 */
  std::vector<oneq::electromagnetics::RfSceneEmission> emitters{}; /**< ESR 辐射源真值 */
  std::vector<electro_optical_sensor::session::EosExternalTargetInput> optical_targets{}; /**< EOS 光学目标 */
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SCENE_TYPES_H_
