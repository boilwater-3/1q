/**
 * @file RuntimeCycleExecutor.h
 * @brief 定义跨模块复用的单周期运行骨架工具。
 */

#ifndef COMMON_RUNTIME_RUNTIME_CYCLE_EXECUTOR_H_
#define COMMON_RUNTIME_RUNTIME_CYCLE_EXECUTOR_H_

#include <cstdint>

namespace oneq {
namespace internal {
namespace runtime {

/**
 * @brief RuntimeCycleStamp 描述单周期执行时的只读标识。
 */
struct RuntimeCycleStamp {
  std::uint32_t cycle_index{0U}; /**< 当前周期号。 */
  std::uint64_t batch_id{1U};    /**< 当前批次号。 */
};

/**
 * @brief RuntimeCycleState 描述骨架维护的通用执行状态。
 * @tparam OutputT 输出帧类型。
 * @tparam ValidationIssuesT 校验问题列表类型。
 */
template <typename OutputT, typename ValidationIssuesT>
struct RuntimeCycleState {
  OutputT latest_output{};                    /**< 最近一次周期输出。 */
  bool has_latest_output{false};              /**< 是否已有可读输出。 */
  ValidationIssuesT last_validation_issues{}; /**< 最近一次输入校验问题。 */
  std::uint64_t next_batch_id{1U};            /**< 下次执行使用的批次号。 */
};

/**
 * @brief RuntimeValidationResult 描述单周期输入校验结论。
 * @tparam ValidationIssuesT 校验问题列表类型。
 */
template <typename ValidationIssuesT>
struct RuntimeValidationResult {
  ValidationIssuesT issues{}; /**< 结构化校验问题集合。 */
  bool has_error{false};      /**< 是否包含阻断执行的错误。 */
};

/**
 * @brief NoValidationIssues 表示无需记录结构化校验问题。
 */
struct NoValidationIssues {};

/**
 * @brief 构造无错误的“空校验”结果。
 * @return 默认通过的校验结论。
 */
inline RuntimeValidationResult<NoValidationIssues> MakePassValidationResult() {
  RuntimeValidationResult<NoValidationIssues> result;
  result.has_error = false;
  return result;
}

/**
 * @brief 构造单周期标识。
 * @param[in] cycle_index 当前周期号。
 * @param[in] batch_id 当前批次号。
 * @return 只读周期标识。
 */
inline RuntimeCycleStamp MakeRuntimeCycleStamp(std::uint32_t cycle_index, std::uint64_t batch_id) {
  RuntimeCycleStamp stamp;
  stamp.cycle_index = cycle_index;
  stamp.batch_id = batch_id;
  return stamp;
}

/**
 * @brief 推进批次号。
 * @param[in,out] batch_id 待推进的批次号。
 */
inline void AdvanceBatchId(std::uint64_t* batch_id) {
  if (batch_id == nullptr) {
    return;
  }
  ++(*batch_id);
}

/**
 * @brief 执行通用单周期骨架流程。
 * @tparam InputT 周期输入类型。
 * @tparam OutputT 周期输出类型。
 * @tparam ValidationIssuesT 校验问题列表类型。
 * @tparam HooksT 业务 Hook 实现类型。
 * @param[in] input 当前周期输入。
 * @param[in] cycle_index 当前周期号。
 * @param[in,out] state 骨架状态缓存。
 * @param[in,out] hooks 业务 Hook。
 *
 * @note `hooks` 需要提供以下成员函数：
 * 1. `Validate(input) -> RuntimeValidationResult<ValidationIssuesT>`
 * 2. `FreezeEnvironment(input, stamp)`
 * 3. `Execute(input, stamp) -> OutputT`
 * 4. `BuildErrorOutput(input, stamp) -> OutputT`
 */
template <typename InputT, typename OutputT, typename ValidationIssuesT, typename HooksT>
void ExecuteRuntimeCycle(const InputT& input, std::uint32_t cycle_index,
                         RuntimeCycleState<OutputT, ValidationIssuesT>* state, HooksT* hooks) {
  if (state == nullptr || hooks == nullptr) {
    return;
  }

  const RuntimeCycleStamp stamp = MakeRuntimeCycleStamp(cycle_index, state->next_batch_id);
  const RuntimeValidationResult<ValidationIssuesT> validation = hooks->Validate(input);
  state->last_validation_issues = validation.issues;

  if (validation.has_error) {
    state->latest_output = hooks->BuildErrorOutput(input, stamp);
    state->has_latest_output = true;
    AdvanceBatchId(&state->next_batch_id);
    return;
  }

  hooks->FreezeEnvironment(input, stamp);
  state->latest_output = hooks->Execute(input, stamp);
  state->has_latest_output = true;
  AdvanceBatchId(&state->next_batch_id);
}

}  // namespace runtime
}  // namespace internal
}  // namespace oneq

#endif  // COMMON_RUNTIME_RUNTIME_CYCLE_EXECUTOR_H_
