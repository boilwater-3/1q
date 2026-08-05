/**
 * @file flight_system.h
 * @brief 飞行系统（平台动力学驱动）：RoutePlan → FlightManager 适配（消费方职责）。
 *
 * flight_dynamic 的接入收敛在本文件的实现（flight_system.cpp）：装配层经
 * CreateFlightDynamics 创建不透明持有者并挂入 registry ctx；flight_system
 * 每周期驱动长机平台并把位姿同步到传感器实体。FD 关闭或初始化失败时回退
 * 运动学推进，assembly/components 对 FD 零依赖。
 */

#ifndef EXAMPLES_BEHAVIOR_LAYER_FLIGHT_SYSTEM_H_
#define EXAMPLES_BEHAVIOR_LAYER_FLIGHT_SYSTEM_H_

#include <entt/entt.hpp>

#include "components.h"

namespace behavior_layer {

/** @brief 飞行动力学持有者（不透明：定义收敛在 flight_system.cpp）。 */
struct FlightDynamicsHolder;

/**
 * @brief 创建飞行动力学持有者并挂入 registry ctx（消费方装配职责）。
 * @note ONEQ_ENABLE_FLIGHT_DYNAMIC 关闭或初始化失败（aircraft 数据缺失/配平失败）
 *       时不创建，flight_system 回退运动学推进。初始位置/速度/航向取自编队状态。
 */
void CreateFlightDynamics(entt::registry& registry, const FleetStatusComponent& initial_fleet);

/** @brief 读取当前飞行动力学持有者（未创建时返回 nullptr）。 */
FlightDynamicsHolder* GetFlightDynamics(entt::registry& registry);

/** @brief 返回飞行器巡航速度（m/s，来自 JSBSim 性能面；未创建时为 0）。 */
double FlightCruiseSpeedMps(const FlightDynamicsHolder& holder);

}  // namespace behavior_layer

#endif  // EXAMPLES_BEHAVIOR_LAYER_FLIGHT_SYSTEM_H_
