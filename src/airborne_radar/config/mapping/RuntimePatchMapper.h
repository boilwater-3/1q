/**
 * @file RuntimePatchMapper.h
 * @brief 定义运行期补丁映射与执行配置反向映射入口。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_MAPPING_RUNTIME_PATCH_MAPPER_H_
#define AIRBORNE_RADAR_SRC_CONFIG_MAPPING_RUNTIME_PATCH_MAPPER_H_

#include <cstdint>

#include "1q/airborne_radar/config/ArRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/config/ArEnvironmentConfig.h"
#include "airborne_radar/config/InternalExecutionConfig.h"

namespace airborne_radar {
namespace config {
namespace mapping {

/**
 * @brief 指定指令生命周期阶段（限时锁定，会话级跨周期状态）。
 *
 * 状态机：kNone → kPending → kAcquired | kExpired（kAcquired/kExpired 为终态）。
 * 推进规则见 ArSession 内 AdvanceDesignationPhase；任一指定相关 patch 字段
 * 变更（含仅改时长）都会在 ApplyRuntimePatch 中重置为 kNone（新指令）。
 */
enum class DesignationPhase : std::uint8_t {
  kNone = 0,     /**< 未指定/未初始化（新指令待首个处理周期开窗）。 */
  kPending = 1,  /**< 捕获窗口内：等待指定目标 confirmed 航迹。 */
  kAcquired = 2, /**< 窗口内已捕获：跟随航迹，不再受窗口限制（后续丢失按回退语义）。 */
  kExpired = 3   /**< 窗口耗尽仍未捕获：指令作废，回到扫描（终态，直到外部重新指定）。 */
};

/**
 * @brief RuntimeConfigState 描述会话持有的运行期配置唯一真值。
 */
struct RuntimeConfigState {
  execution::InternalExecutionConfig execution_config{};
  config::EnvironmentScenarioConfig environment_scenario_config{};
  config::AzimuthElevationDeg dwell_center_deg{};
  /**
   * @brief 指定跟踪目标外部 ID（STT 航迹跟随指向；0 = 未指定/已清除）。
   * @note 会话级状态：不进 pipeline 执行配置（pipeline 不消费指向来源），
   *       仅由 ArSession prepare 阶段消费；随 patch 原子暂存/提交/回滚。
   */
  std::uint64_t designated_external_target_id{0U};
  /**
   * @brief 限时指令窗口周期数（0 = 无限期，旧行为；见 ArRuntimeConfigPatch
   *        designation_duration_cycles 注释）。
   */
  std::uint32_t designation_duration_cycles{0U};
  /** @brief 指定指令生命周期阶段（跨周期状态；随 patch 原子暂存/提交/回滚）。 */
  DesignationPhase designation_phase{DesignationPhase::kNone};
  /**
   * @brief 捕获窗口截止周期（指令生效后首个处理周期 + duration；0 = 无限期）。
   * @note 作废沿发生在 cycle == 本值的周期；作废后本值保留供沿识别。
   */
  std::uint32_t designation_deadline_cycle_index{0U};
};

/**
 * @brief RuntimeConfigResolveResult 描述运行期补丁解析结果与执行计划。
 */
struct RuntimeConfigResolveResult {
  RuntimeConfigState next_state{};
  bool has_requested_update{false};
  bool is_valid{true};
  bool execution_config_changed{false};
  bool environment_scenario_config_changed{false};
};

/**
 * @brief 解析运行期补丁并生成会话运行态更新计划。
 * @param[in] current_state 当前运行态。
 * @param[in] patch 外部提交补丁。
 * @return 解析结果与更新计划。
 */
RuntimeConfigResolveResult ApplyRuntimePatch(const RuntimeConfigState& current_state,
                                             const ArRuntimeConfigPatch& patch);

/**
 * @brief 将内部执行配置反向映射为四域会话配置。
 * @param[in] execution_config 内部执行配置。
 * @return 对应的四域会话配置。
 */
config::ArSessionConfig MapExecutionToSession(
    const execution::InternalExecutionConfig& execution_config);

/**
 * @brief 将运行期状态映射为当前 pipeline 可消费的四域会话配置。
 * @param[in] runtime_state 当前运行期状态。
 * @return 已叠加运行期驻留偏移的四域会话配置。
 */
config::ArSessionConfig MapRuntimeStateToPipelineSession(
    const RuntimeConfigState& runtime_state);

}  // namespace mapping
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_MAPPING_RUNTIME_PATCH_MAPPER_H_
