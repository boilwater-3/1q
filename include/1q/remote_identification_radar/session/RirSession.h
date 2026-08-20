/**
 * @file RirSession.h
 * @brief 定义面向外部接入的远程识别雷达会话门面。
 *
 * 会话门面（一步一帧执行、运行期补丁）的主头文件。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_SESSION_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/foundation/SensorContract.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/config/RirSessionConfigValidation.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirOutputTypes.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"

namespace remote_identification_radar {
namespace config {
struct RirRuntimeConfigPatch;
}  // namespace config
}  // namespace remote_identification_radar

namespace remote_identification_radar {
namespace session {

/**
 * @brief RirSession 提供"一步一帧"的外部接入门面。
 */
class ONEQ_API RirSession {
 public:
  RirSession();
  ~RirSession();

  RirSession(const RirSession&) = delete;
  RirSession& operator=(const RirSession&) = delete;
  RirSession(RirSession&&) noexcept;
  RirSession& operator=(RirSession&&) noexcept;

  /**
   * @brief 执行一个识别处理周期。
   * @param[in] input 当前周期输入。
   * @return 当前周期生成的识别输出帧拷贝。
   * @note 非法或被拒绝的周期返回默认的当前帧，不复用历史输出。
   */
  RirOutputFrame Step(const RirCycleInput& input);

  /**
   * @brief 执行一个识别处理周期，并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   * @note `status` 是结果有效性的唯一真相；只有 `kCompleted` 携带本周期识别输出。
   */
  RirCycleResult StepWithResult(const RirCycleInput& input);

  /**
   * @brief 尝试应用运行期可变配置补丁。
   * @param[in] patch 运行期可变配置补丁。
   * @note 未设置的字段保持现值不变；补丁在下一个成功执行周期边界提交。
   * @return 补丁被接受并暂存成功时返回 true；补丁无效或无变更时返回 false。
   */
  bool TryApplyRuntimeConfig(const config::RirRuntimeConfigPatch& patch);

  /** @brief 最近周期是否发布了识别效能摘要。 */
  bool HasLatestRecognitionSummary() const;

  /** @brief 获取最近周期识别效能摘要。 */
  const RirRecognitionCycleSummary& GetLatestRecognitionSummary() const;

  /** @brief 使用四域配置创建会话（推荐入口，信任路径，不做配置校验）。 */
  static RirSession Create(const config::RirSessionConfig& config = {});

  /**
   * @brief 创建会话并报告配置校验结果（校验路径）。
   * @param[in] config 四域会话配置。
   * @param[out] issues 校验问题输出；传入 nullptr 则不写回但仍构造会话。
   * @return 构造完成的会话。
   * @note 无论 @p issues 是否为空都会构造并返回会话（不阻断）。
   */
  static RirSession CreateWithDiagnostics(const config::RirSessionConfig& config,
                                          RirIssueList* issues);

 private:
  friend class RirSessionReplayAccess;

  struct Impl;
  explicit RirSession(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace remote_identification_radar

// 跨域传感器会话形状契约：锚定 Step/StepWithResult 签名。
ONEQ_SENSOR_SESSION_CONTRACT(remote_identification_radar::session::RirSession,
                             remote_identification_radar::session::RirCycleInput,
                             remote_identification_radar::session::RirOutputFrame,
                             remote_identification_radar::session::RirCycleResult);

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_SESSION_H_
