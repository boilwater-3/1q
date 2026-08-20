/**
 * @file PreparedCycleLedger.h
 * @brief AR 周期账本——拥有 prepared-cycle 书记、编年史与单调计数器/令牌。
 *
 * 提供显式的快照/恢复语义：RunCycle 在周期顶部快照本对象，PrepareRfCycle 失败时
 * 把快照赋回账本即可逐字段回滚（emission ID、令牌、prepare 计数、编年史等均不变）。
 * 与 ECM 的 EcmResourceLedger 同型：自包含、可拷贝值类型，使事务回滚坍缩为单次赋值。
 */

#ifndef AIRBORNE_RADAR_SESSION_PREPARED_CYCLE_LEDGER_H_
#define AIRBORNE_RADAR_SESSION_PREPARED_CYCLE_LEDGER_H_

#include <cstddef>
#include <cstdint>

#include "1q/electromagnetics/RfScene.h"
#include "airborne_radar/session/ArRfCycleState.h"

namespace airborne_radar {
namespace session {

/**
 * @brief 拥有单周期 prepared-cycle 全部书记状态：编年史、令牌与单调计数器、冻结的
 * prepare 输入/发射/接收工作状态。
 *
 * 事务模式：RunCycle 在顶部 `auto saved = ledger;`，PrepareRfCycle 失败后
 * `ledger = saved;` 回滚。成功 prepare 由 CommitPrepared 落定；Complete 成功后
 * ClearPrepared 释放待完成状态；Abandon 释放 PrepareEmissionControl 的控制器侧占用。
 */
class PreparedCycleLedger {
 public:
  // --- 编年史（world time 单调推进）---

  bool has_world_chronology() const { return has_world_chronology_; }
  double last_world_window_end_s() const { return last_world_window_end_s_; }

  /// @brief 记录世界时间已推进到 window 末尾（powered-off 与成功 prepare 共用）。
  void AdvanceWorldChronology(double window_start_time_s, double window_duration_s) {
    has_world_chronology_ = true;
    last_world_window_end_s_ = window_start_time_s + window_duration_s;
  }

  // --- 单调计数器与令牌（只读访问）---

  std::uint64_t next_token_value() const { return next_token_value_; }
  std::uint64_t next_emission_id() const { return next_emission_id_; }
  std::uint64_t successful_prepare_count() const { return successful_prepare_count_; }
  std::size_t frequency_hop_index() const { return frequency_hop_index_; }

  // --- Prepared-cycle 状态（冻结于 prepare，消费于 complete）---

  bool has_prepared_cycle() const { return has_prepared_cycle_; }
  const ArPreparedCycleToken& prepared_token() const { return prepared_token_; }
  const ArPrepareCycleInput& prepared_input() const { return prepared_input_; }
  const oneq::electromagnetics::RfSceneEmission& prepared_emission() const {
    return prepared_emission_;
  }
  const ArReceiverOperatingState& prepared_operating_state() const {
    return prepared_operating_state_;
  }

  /// @brief 令牌是否属于当前待完成周期（RunCycle 总传入刚铸造的令牌）。
  bool TokenMatches(const ArPreparedCycleToken& token) const {
    return has_prepared_cycle_ && token.value != 0U && token.value == prepared_token_.value &&
           token.world_cycle_index == prepared_token_.world_cycle_index;
  }

  /**
   * @brief 落定一次成功的 prepare：铸造令牌、冻结输入/发射/接收状态、推进编年史与计数器。
   *
   * 原子提交：调用前账本未被修改，调用后全部字段同时更新。失败路径由调用方放弃候选，
   * 账本保持周期开始前的快照值。
   */
  void CommitPrepared(const ArPrepareCycleInput& input,
                      const oneq::electromagnetics::RfSceneEmission& emission,
                      const ArReceiverOperatingState& operating_state,
                      std::size_t committed_frequency_hop_index) {
    prepared_token_.value = next_token_value_++;
    prepared_token_.world_cycle_index = input.world_cycle_index;
    prepared_input_ = input;
    prepared_emission_ = emission;
    prepared_operating_state_ = operating_state;
    has_prepared_cycle_ = true;
    AdvanceWorldChronology(input.window_start_time_s, input.window_duration_s);
    ++next_emission_id_;
    ++successful_prepare_count_;
    frequency_hop_index_ = committed_frequency_hop_index;
  }

  /// @brief Complete 成功后释放待完成状态（令牌归零，计数器/编年史保留）。
  void ClearPrepared() {
    has_prepared_cycle_ = false;
    prepared_token_ = ArPreparedCycleToken{};
  }

  /// @brief Abandon 后释放待完成状态（与 ClearPrepared 相同的书记清理）。
  void ReleasePrepared() { ClearPrepared(); }

 private:
  bool has_world_chronology_{false};
  double last_world_window_end_s_{0.0};
  std::uint64_t next_token_value_{1U};
  std::uint64_t next_emission_id_{1U};
  std::uint64_t successful_prepare_count_{0U};
  std::size_t frequency_hop_index_{0U};
  bool has_prepared_cycle_{false};
  ArPreparedCycleToken prepared_token_{};
  ArPrepareCycleInput prepared_input_{};
  oneq::electromagnetics::RfSceneEmission prepared_emission_{};
  ArReceiverOperatingState prepared_operating_state_{};
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_PREPARED_CYCLE_LEDGER_H_
