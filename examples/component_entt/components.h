/**
 * @file components.h
 * @brief 行为层 EnTT 数据组件集合。
 *
 * 组件为纯数据（无虚函数、无逻辑），逻辑由 systems 以自由函数承担；
 * 实体/组件装配由消费方 EnTT registry 承担，组件组合方式见
 * docs/review/Bahavior.md §5（EnTT 数据组件 + 系统，替代多态集合方案）。
 */

#ifndef EXAMPLES_BEHAVIOR_LAYER_COMPONENTS_H_
#define EXAMPLES_BEHAVIOR_LAYER_COMPONENTS_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <entt/entt.hpp>

#include "1q/airborne_radar/session/ArCommand.h"
#include "1q/airborne_radar/session/DecisionControlTypes.h"
#include "1q/coordinate/types.h"
#include "1q/electronic_countermeasure/EcmTypes.h"
#include "1q/fusion/DetectionRecord.h"
#include "1q/fusion/FusedTarget.h"
#include "1q/navigation/CoverageArea.h"
#include "1q/navigation/CoveragePlanConfig.h"
#include "1q/navigation/RoutePoint.h"

namespace component_entt {

/**
 * @brief 平台任务角色。
 */
enum class Role {
  kSingle = 0, /**< 单机：无上级无下级，自主规划自身区域任务 */
  kLead,       /**< 长机：无上级有下级，规划全员航路 */
  kWingman,    /**< 僚机：有上级，被动零计算 */
};

/** @brief 演示源通道标识（与融合配置 source_weights 索引一致；索引 0 未用）。 */
constexpr std::uint32_t kArSourceId = 1U;  /**< AR 源通道 */
constexpr std::uint32_t kEsrSourceId = 2U; /**< ESR 源通道 */
constexpr std::uint32_t kEosSourceId = 3U; /**< EOS 源通道 */

/**
 * @brief 任务组件：角色、上下级与区域任务。
 * @note 层级来源为显式 Tasking 输入（确定性仿真无"发现"机制，冻结契约 §5）；
 *       区域任务载荷复用库内 navigation 算法面类型。
 */
struct TaskingComponent {
  Role role{Role::kSingle};                 /**< 任务角色 */
  entt::entity superior{entt::null};        /**< 上级实体（kSingle/kLead 时为 entt::null） */
  std::vector<entt::entity> subordinates{}; /**< 下级实体列表（长机填充） */
  navigation::CoverageArea region{};        /**< 覆盖区域任务 */
  navigation::CoveragePlanConfig region_config{}; /**< 覆盖规划参数 */
};

/**
 * @brief 传感器观测组件：会话输出适配后的泛型探测记录。
 * @note source_id 与融合配置 source_weights 的索引语义一致（缺省按 1.0 计）；
 *       探测记录复用库内 fusion::DetectionRecord（算法面传感器无关）。
 */
struct SensorObservationComponent {
  std::uint32_t source_id{0U};                       /**< 源通道标识 */
  std::vector<fusion::DetectionRecord> detections{}; /**< 本周期适配后的探测记录 */
};

/**
 * @brief 编队状态组件：消费方聚合注入的平台状态（位置/航向/速度）。
 * @note 库内导航/融合面不感知编队概念；编队编排属业务层（冻结契约 §7）。
 */
struct FleetStatusComponent {
  std::uint64_t platform_entity_id{0U};           /**< 平台实体标识 */
  oneq::coordinate::LlaPositionDegM position{};   /**< 平台位置（度制 LLA） */
  double heading_deg{0.0};                        /**< 航向（单位：deg，北偏东） */
  double speed_mps{0.0};                          /**< 速度（单位：m/s） */
};

/**
 * @brief 航路计划组件：长机/单机规划产物。
 * @note version 随每次重规划递增，供消费方做变化检测（如触发指令下发）。
 */
struct RoutePlanComponent {
  navigation::RoutePlan route{}; /**< 航路计划（相邻航点直线航段，驱动属消费方职责） */
  std::size_t next_index{0U};    /**< 下一航点索引（消费方驱动） */
  std::uint32_t version{0U};     /**< 计划版本（每次重规划递增） */
};

/**
 * @brief 融合态势组件：融合引擎输出与事件计数。
 * @note new/lost 计数供观察者做事件触发报告（报告节奏属业务层，见冻结契约 §4.2）。
 */
struct FusedSituationComponent {
  std::vector<fusion::FusedTarget> targets{}; /**< 当前融合目标态势（按 key 升序） */
  std::size_t new_target_count{0U};           /**< 本周期新目标数 */
  std::size_t lost_target_count{0U};          /**< 本周期消失目标数 */
};

/**
 * @brief 命令帧组件：决策系统聚合输出。
 * @note 事件模型：命令 = 写入本组件，消费方读取后驱动各执行面（
 *       AR SubmitExternalDecision / ECM EcmCycleInput / flight_dynamic），
 *       不建全局事件总线（冻结契约 §5）。
 */
struct CommandFrameComponent {
  std::vector<airborne_radar::session::ArCommand> ar_commands{}; /**< AR 战术指令 */
  std::vector<electronic_countermeasure::session::EcmCycleInput> ecm_inputs{}; /**< ECM 周期输入（jam 接线位） */
  airborne_radar::session::ExternalDecisionOverride external_decision{}; /**< 外部 profile 覆盖（可选） */
  bool has_external_decision{false}; /**< 本周期是否携带外部决策覆盖 */
};

}  // namespace component_entt

#endif  // EXAMPLES_BEHAVIOR_LAYER_COMPONENTS_H_
