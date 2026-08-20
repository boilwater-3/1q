/**
 * @file ControlReducer.h
 * @brief 定义控制意图到控制真值的私有归并器实现。
 */

#ifndef AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_H_
#define AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_H_

#include <cstdint>

#include "airborne_radar/decision/ControlReducerTypes.h"
#include "airborne_radar/decision/ControlReducerTypes.h"

namespace airborne_radar {
namespace decision {

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
   * @param[in] config reducer 配置。
   */
  explicit ControlReducer(extension::ControlReducerConfig config = {});

  /**
   * @brief 更新 reducer 配置。
   * @param[in] config 新的 reducer 配置。
   * @note 当前 hold/cooldown 剩余周期会收紧到新上限；增大配置不会延长已开始的窗口。
   */
  void UpdateConfig(extension::ControlReducerConfig config);

  /**
   * @brief 获取当前 reducer 配置。
   * @return 当前 reducer 配置副本。
   */
  extension::ControlReducerConfig GetConfig() const;

  /**
   * @brief 判断控制意图是否属于 LPI 域（归约权威判定）。
   * @param[in] type 控制意图类型。
   * @return 属于 LPI 域时返回 true。
   * @note reducer 与外部提案校验必须共用此判定，避免重复实现漂移。
   */
  static bool IsLpiDirective(session::ControlDirectiveType type);

  /**
   * @brief 判断控制意图是否属于 ECCM 域（归约权威判定）。
   * @param[in] type 控制意图类型。
   * @return 属于 ECCM 域时返回 true。
   * @note reducer 与外部提案校验必须共用此判定，避免重复实现漂移。
   */
  static bool IsEccmDirective(session::ControlDirectiveType type);

  /**
   * @brief 校验单条控制意图的标量合法性（归约权威判定）。
   * @param[in] directive 待校验的控制意图。
   * @return 合法时返回 true。
   * @note 标量 directive（power/dwell/burnthrough）校验取值范围，
   *       布尔 directive 要求不携带标量。
   */
  static bool IsValidDirectiveValue(const session::ControlDirective& directive);

  /**
   * @brief 使用上一版 profile 和 proposal 列表生成下一版 profile。
   * @param[in] previous_profile 上一版控制真值。
   * @param[in] proposals 当前周期候选控制意图列表。
   * @return 归并后的控制真值与采纳结果。
   */
  extension::ControlReductionResult Reduce(
      const session::ArControlProfile& previous_profile,
      const std::vector<session::TacticalProposal>& proposals);

  /**
   * @brief 获取 reducer 当前内部运行态（供调试/测试使用）。
   * @return 当前 reducer 运行态快照，仅供调试与测试断言使用。
   */
  ControlReducerRuntimeState GetRuntimeState() const;

  /** @brief 恢复此前捕获的 reducer 跨周期运行态。 */
  void RestoreRuntimeState(const ControlReducerRuntimeState& state);

 private:
  extension::ControlReducerConfig config_{};

  /** @brief LPI 域剩余保持周期（reducer 内部运行态）。 */
  std::uint32_t lpi_hold_cycles_remaining_{0};
  /** @brief ECCM 域剩余保持周期（reducer 内部运行态）。 */
  std::uint32_t eccm_hold_cycles_remaining_{0};
  /** @brief LPI 域释放后的冷却周期（reducer 内部运行态）。 */
  std::uint32_t lpi_cooldown_cycles_remaining_{0};
  /** @brief ECCM 域释放后的冷却周期（reducer 内部运行态）。 */
  std::uint32_t eccm_cooldown_cycles_remaining_{0};
};

}  // namespace decision
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_H_
