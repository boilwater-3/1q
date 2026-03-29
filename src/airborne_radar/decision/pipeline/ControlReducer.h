/**
 * @file ControlReducer.h
 * @brief 定义控制意图到控制真值的私有归并器实现。
 */

#ifndef AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_H_
#define AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_H_

#include <cstdint>

#include "1q/airborne_radar/decision/pipeline/ControlReducerTypes.h"
#include "1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h"

namespace airborne_radar {
namespace decision {
namespace pipeline {

/**
 * @brief ControlReducerRuntimeState 存储 reducer 跨周期的内部运行态。
 * @note 仅供调试和测试使用；外部消费者不应依赖此结构做运行时决策。
 */
struct ControlReducerRuntimeState {
  std::uint32_t lpi_hold_cycles_remaining{0};
  std::uint32_t eccm_hold_cycles_remaining{0};
  std::uint32_t lpi_cooldown_cycles_remaining{0};
  std::uint32_t eccm_cooldown_cycles_remaining{0};
};

/**
 * @brief 负责把控制意图归并为唯一控制真值。
 */
class ControlReducer {
 public:
  /**
   * @brief 使用配置构造 reducer。
   * @param config reducer 配置。
   */
  explicit ControlReducer(ControlReducerConfig config = {});

  /**
   * @brief 更新 reducer 配置。
   * @param config 新的 reducer 配置。
   */
  void UpdateConfig(ControlReducerConfig config);

  /**
   * @brief 获取当前 reducer 配置。
   * @return 当前 reducer 配置副本。
   */
  ControlReducerConfig GetConfig() const;

  /**
   * @brief 使用上一版 profile 和 proposal 列表生成下一版 profile。
   * @param previous_profile 上一版控制真值。
   * @param proposals 当前周期候选控制意图列表。
   * @return 归并后的控制真值与采纳结果。
   */
  ControlReductionResult Reduce(const common::control::RadarControlProfile& previous_profile,
                                const std::vector<TacticalProposal>& proposals);

  /**
   * @brief 获取 reducer 当前内部运行态（供调试/测试使用）。
   */
  ControlReducerRuntimeState GetRuntimeState() const;

 private:
  ControlReducerConfig config_{};

  /** @brief LPI 域剩余保持周期（reducer 内部运行态）。 */
  std::uint32_t lpi_hold_cycles_remaining_{0};
  /** @brief ECCM 域剩余保持周期（reducer 内部运行态）。 */
  std::uint32_t eccm_hold_cycles_remaining_{0};
  /** @brief LPI 域释放后的冷却周期（reducer 内部运行态）。 */
  std::uint32_t lpi_cooldown_cycles_remaining_{0};
  /** @brief ECCM 域释放后的冷却周期（reducer 内部运行态）。 */
  std::uint32_t eccm_cooldown_cycles_remaining_{0};
};

}  // namespace pipeline
}  // namespace decision
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_H_
