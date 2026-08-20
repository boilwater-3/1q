/**
 * @file EosExclusionCauseRecorder.h
 * @brief EOS 排除原因跨周期差分记录器（规则 13b 排除诊断的差分观测）。
 *
 * 对持续被排除的目标，当其排除原因（code + cause 组合对）跨周期变化时产生结构化事件，
 * 与既有探测生命周期事件并列（独立 recorder、独立 `GetLastEvents()` 通道）。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_EXCLUSION_CAUSE_RECORDER_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_EXCLUSION_CAUSE_RECORDER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"

namespace electro_optical_sensor {
namespace session {

struct EosCycleInput;

/**
 * @brief 排除原因跨周期变化事件类型（需求文档 §2.3 转换语义）。
 *
 * A1（原因稳定）不产生事件。
 */
enum class ONEQ_API EosExclusionCauseEventKind {
  kEntered = 0, /**< A2：目标从未被排除（kNone）进入被排除。 */
  kChanged = 1, /**< A3：被排除目标的 (code,cause) 组合对发生变化（本需求核心）。 */
  kExited = 2   /**< A4：目标从被排除恢复为未被排除（kNone）。 */
};

/**
 * @brief 单条排除原因跨周期变化事件记录。
 *
 * `previous_*` 为空（code）/ kNone（cause）表示上一周期未被排除（A2）；
 * `current_*` 为空（code）/ kNone（cause）表示本周期不再被排除（A4）。
 */
struct ONEQ_API EosExclusionCauseEvent {
  std::uint32_t cycle_index{0U};                                    /**< 触发该事件的周期序号 */
  std::uint64_t target_id{0U};                                      /**< 目标 ID */
  std::string target_name{};                                        /**< 目标名称（人读标签） */
  EosExclusionCauseEventKind kind{EosExclusionCauseEventKind::kChanged}; /**< 事件类型 */
  std::string previous_code{};                                      /**< 上一执行周期排除 code（空 = 上周未被排除） */
  EosIssueCause previous_cause{EosIssueCause::kNone};               /**< 上一执行周期排除主因 */
  std::string current_code{};                                       /**< 本执行周期排除 code（空 = 本周不再被排除） */
  EosIssueCause current_cause{EosIssueCause::kNone};                /**< 本执行周期排除主因 */
};

/**
 * @brief 排除原因跨周期差分记录器。
 *
 * 转换检测状态机（非数据存储）：累积状态刻意最小化为每目标上一执行周期的
 * (code,cause) 组合对（无条目 = 上周未被排除）。非 completed 周期不产生事件，
 * 也不推进记录器状态（与既有 `EosDetectionLifecycleRecorder` 语义一致）。
 *
 * 差分键为 (code,cause) 组合对（与 AR/SBIRS 一致）：EOS 仅有单一排除 code
 * （视场门 `eos.target_out_of_fov`），组合键下等价于纯 cause 差分（越界轴变化）；
 * 保持组合键为跨模块统一约定。
 *
 * 纯观测：只读 `result.issues`（按 `location.kind == kSceneEntity` 关联目标），
 * 不改变 `*CycleStatus`、排除诊断、DebugView 状态语义（规则 13c 边界延续）。
 * 私有状态（含 unordered_map）与判定逻辑见 .cpp，避免在 public header 暴露实现细节。
 *
 * @note 单目标单周期多条排除 issue 的假设：当前 EOS 仅单一视场门排除，
 *       取按 location 命中的第一条；若未来门并发需 revisit。
 */
class ONEQ_API EosExclusionCauseRecorder {
 public:
  EosExclusionCauseRecorder();
  ~EosExclusionCauseRecorder();

  EosExclusionCauseRecorder(const EosExclusionCauseRecorder&) = delete;
  EosExclusionCauseRecorder& operator=(const EosExclusionCauseRecorder&) = delete;
  EosExclusionCauseRecorder(EosExclusionCauseRecorder&&) noexcept;
  EosExclusionCauseRecorder& operator=(EosExclusionCauseRecorder&&) noexcept;

  /**
   * @brief 基于目标事实与单周期结果产出排除原因跨周期变化事件。
   *
   * 仅处理 `target_id != 0` 的输入目标；按 A2/A3/A4 转换规则生成事件，
   * A1（原因稳定）不产生事件。差分原料取自 `result.issues` 中
   * `location.kind == kSceneEntity` 的排除诊断条目（按 `entity_index` 关联目标）。
   *
   * @param[in] input 当前周期输入（用于遍历场景目标表与回查 target_name）。
   * @param[in] result 当前周期结果（用于读取排除诊断与周期号）。
   * @return 本周期产生的排除原因变化事件列表（可能为空）。
   */
  std::vector<EosExclusionCauseEvent> Update(const EosCycleInput& input,
                                             const EosCycleResult& result);

  /**
   * @brief 清空内部目标排除状态，回到初始状态。
   */
  void Reset();

  /**
   * @brief 获取最近一次执行周期 `Update()` 返回的事件列表。
   *
   * 事件在每次执行周期的 `Update()` 调用时缓存；非执行周期不刷新缓存，
   * 保留上一次执行周期的事件。供注册到 Session 后由调用方事后读取。
   * @return 最近一次执行周期 `Update()` 产生的事件列表的 const 引用。
   */
  const std::vector<EosExclusionCauseEvent>& GetLastEvents() const noexcept;

 private:
  // 不透明私有状态，定义在 .cpp 中，避免在 header 暴露 <unordered_map> 依赖。
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_EXCLUSION_CAUSE_RECORDER_H_
