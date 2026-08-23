/**
 * @file RirExclusionCauseRecorder.h
 * @brief 远程识别雷达排除原因跨周期差分记录器（规则 13b 排除诊断的差分观测）。
 *
 * 对持续被排除的目标，当其排除原因（code + cause 组合对）跨周期变化时产生结构化事件，
 * 与既有航迹生命周期事件并列（独立 recorder、独立 `GetLastEvents()` 通道）。
 * 观测投影契约见 docs/review/rir_observability_projections_freeze_2026-08-21.md §3.4。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_EXCLUSION_CAUSE_RECORDER_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_EXCLUSION_CAUSE_RECORDER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirSceneTypes.h"

namespace remote_identification_radar {
namespace session {

/**
 * @brief 排除原因跨周期变化事件类型（需求文档 §2.3 转换语义）。
 *
 * A1（原因稳定）不产生事件。
 */
enum class ONEQ_API RirExclusionCauseEventKind {
  kEntered = 0, /**< A2：目标从未被排除（kNone）进入被排除。 */
  kChanged = 1, /**< A3：被排除目标的 (code,cause) 组合对发生变化（本需求核心）。 */
  kExited = 2   /**< A4：目标从被排除恢复为未被排除（kNone）。 */
};

/**
 * @brief 单条排除原因跨周期变化事件记录。
 *
 * `previous_*` 为空（code）/ kNone（cause）表示上一执行周期未被排除（A2）；
 * `current_*` 为空（code）/ kNone（cause）表示本执行周期不再被排除（A4）。
 */
struct ONEQ_API RirExclusionCauseEvent {
  std::uint32_t world_cycle_index{0U}; /**< 触发该事件的输入周期号 */
  std::uint64_t external_target_id{0U}; /**< 输入目标外部标识符 */
  std::string target_name{};            /**< 目标名称（人读标签） */
  RirExclusionCauseEventKind kind{RirExclusionCauseEventKind::kChanged}; /**< 事件类型 */
  std::string previous_code{};          /**< 上一执行周期排除 code（空 = 上周未被排除） */
  RirIssueCause previous_cause{RirIssueCause::kNone}; /**< 上一执行周期排除主因 */
  std::string current_code{};           /**< 本执行周期排除 code（空 = 本周不再被排除） */
  RirIssueCause current_cause{RirIssueCause::kNone};  /**< 本执行周期排除主因 */
};

/**
 * @brief 排除原因跨周期差分记录器。
 *
 * 转换检测状态机（非数据存储）：累积状态刻意最小化为每目标上一执行周期的
 * (code,cause) 组合对（无条目 = 上周未被排除）。非 completed 周期不产生事件，
 * 也不推进记录器状态（与既有 `RirTrackLifecycleRecorder` 语义一致）。
 *
 * 差分键为 (code,cause) 组合对（而非纯 cause）：避免"同为 kNone 的具体门切换
 * 盲区"（RIR 多门：检测门/识别距离门/模式门/特征库门）。
 *
 * 纯观测：只读 `result.issues`（按 `location.kind == kSceneEntity` 关联目标），
 * 不改变 `*CycleStatus`、排除诊断、DebugView 状态语义（规则 13c 边界延续）。
 * 私有状态（含 unordered_map）与判定逻辑见 .cpp，避免在 public header 暴露实现细节。
 *
 * @note 单目标单周期多条排除 issue 的假设：RIR 执行链每目标每周期至多落一条
 *       排除诊断（链上第一门优先），取按 location 命中的第一条。
 */
class ONEQ_API RirExclusionCauseRecorder {
 public:
  RirExclusionCauseRecorder();
  ~RirExclusionCauseRecorder();

  RirExclusionCauseRecorder(const RirExclusionCauseRecorder&) = delete;
  RirExclusionCauseRecorder& operator=(const RirExclusionCauseRecorder&) = delete;
  // 移动操作声明在 header、定义在 .cpp：unique_ptr<Impl> 析构需要完整类型，
  // 不能内联定义否则破坏 PImpl 不透明性。
  RirExclusionCauseRecorder(RirExclusionCauseRecorder&&) noexcept;
  RirExclusionCauseRecorder& operator=(RirExclusionCauseRecorder&&) noexcept;

  /**
   * @brief 基于场景目标事实与单周期结果产出排除原因跨周期变化事件。
   *
   * 仅处理 `external_target_id != 0` 的输入目标；按 A2/A3/A4 转换规则生成事件，
   * A1（原因稳定）不产生事件。差分原料取自 `result.issues` 中
   * `location.kind == kSceneEntity` 的排除诊断条目（按 `entity_index` 关联目标）。
   *
   * @param[in] targets 当前周期场景目标事实（用于遍历输入目标表与回查 target_name）。
   * @param[in] result 当前周期结果（用于读取排除诊断与周期号）。
   * @return 本周期产生的排除原因变化事件列表（可能为空）。
   */
  std::vector<RirExclusionCauseEvent> Update(const RirSceneTargetList& targets,
                                             const RirCycleResult& result);

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
  const std::vector<RirExclusionCauseEvent>& GetLastEvents() const noexcept;

 private:
  // 不透明私有状态，定义在 .cpp 中，避免在 header 暴露 <unordered_map> 依赖。
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_EXCLUSION_CAUSE_RECORDER_H_
