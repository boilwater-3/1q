/**
 * @file EnvironmentCycleContext.h
 * @brief 定义统一的环境层周期冻结上下文。
 */

#ifndef ONEQ_ENVIRONMENT_ENVIRONMENT_CYCLE_CONTEXT_H_
#define ONEQ_ENVIRONMENT_ENVIRONMENT_CYCLE_CONTEXT_H_

#include <cstdint>

#include "1q/api.hpp"

namespace oneq {
namespace environment {

/**
 * @brief EnvironmentCycleContext 描述环境层周期冻结上下文。
 *
 * 统一 AR 的 EnvironmentCycleContext 和 ESR 的 EsrEnvironmentCycleContext 中
 * 的共同字段（cycle_index + dt_sec），供各模块的周期冻结逻辑复用。
 */
struct ONEQ_API EnvironmentCycleContext {
  std::uint32_t cycle_index{0U}; /**< 当前周期号 */
  float dt_sec{0.0f};            /**< 当前周期步长（单位：s） */
};

}  // namespace environment
}  // namespace oneq

#endif  // ONEQ_ENVIRONMENT_ENVIRONMENT_CYCLE_CONTEXT_H_
