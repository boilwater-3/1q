/**
 * @file systems.h
 * @brief 行为层系统声明。
 *
 * 系统为自由函数（entt::registry&），会话与引擎经 registry ctx 中的
 * BehaviorContext 访问；每周期按 flight → recon → maneuver → jam → decision
 * 顺序由 StepBehaviorLayer 调用（对齐 session Step 语义，见冻结契约 §5）。
 */

#ifndef EXAMPLES_BEHAVIOR_LAYER_SYSTEMS_H_
#define EXAMPLES_BEHAVIOR_LAYER_SYSTEMS_H_

#include <entt/entt.hpp>

namespace behavior_layer {

/** @brief 行为层周期时长（s）：三会话周期语义 + 飞行子步进的外层时钟。 */
constexpr double kBehaviorDtSec = 1.0;

/** @brief 飞行系统：驱动长机平台动力学（FD 或运动学回退）并同步传感器实体。 */
void flight_system(entt::registry& registry);

/** @brief 侦察系统：驱动 AR 会话，把轨迹输出适配为泛型探测记录并更新融合态势。 */
void recon_system(entt::registry& registry);

/** @brief 机动规划系统：按任务层级为平台规划区域覆盖航路（长机规划全员）。 */
void maneuver_system(entt::registry& registry);

/** @brief 干扰系统：从编队/融合态势构造 ECM 周期输入（仅 ECM 既有公共面）。 */
void jam_system(entt::registry& registry);

/** @brief 决策系统：聚合融合态势/航路/决策观测产出命令帧。 */
void decision_system(entt::registry& registry);

}  // namespace behavior_layer

#endif  // EXAMPLES_BEHAVIOR_LAYER_SYSTEMS_H_
