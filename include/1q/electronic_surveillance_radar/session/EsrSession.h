/**
 * @file EsrSession.h
 * @brief 定义面向外部调用方的电子侦察会话门面。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SESSION_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SESSION_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "1q/electronic_surveillance_radar/output/EsrOutputFrame.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"

namespace electronic_surveillance_radar {
namespace extension {
class EsrController;
}

namespace extension {
class IInterceptPipeline;
}
namespace environment {
class IEsrEnvironmentService;
}
}  // namespace electronic_surveillance_radar

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief ESR 运行期补丁应用状态码。
 */
enum class EsrRuntimeConfigApplyStatus {
  kApplied = 0,
  kNoRequestedUpdate,
  kRejectedInvalidScanRate,
  kRejectedInvalidScanCenterAz,
  kRejectedInvalidScanCenterEl,
  kRejectedInvalidExplicitScanBounds,
  kRejectedUnsupportedEnvironmentPresetPatch,
};

/**
 * @brief ESR 运行期补丁应用结果。
 */
struct ONEQ_API EsrRuntimeConfigApplyResult {
  EsrRuntimeConfigApplyStatus status{EsrRuntimeConfigApplyStatus::kNoRequestedUpdate};
  bool has_requested_update{false};
  bool applied{false};
};

/**
 * @brief EsrSession 提供单周期外部接入门面。
 * @note 线程模型：会话对象持有可变状态，默认非线程安全；并发访问需外部同步。
 */
class ONEQ_API EsrSession {
 public:
  /**
   * @brief 使用默认装配配置构造会话。
   * @param[in] config 会话配置。
   */
  explicit EsrSession(EsrSessionConfig config = {});
  /**
   * @brief 使用外部装配链路构造会话（引用注入，不接管生命周期）。
   */
  EsrSession(EsrSessionConfig config, extension::IInterceptPipeline& pipeline);
  /**
   * @brief 使用外部装配链路构造会话（引用注入，不接管生命周期）。
   */
  EsrSession(EsrSessionConfig config, environment::IEsrEnvironmentService& environment_service);
  /**
   * @brief 使用外部装配链路构造会话（引用注入，不接管生命周期）。
   */
  EsrSession(EsrSessionConfig config, extension::EsrController& controller);
  /**
   * @brief 使用外部装配链路构造会话（引用注入，不接管生命周期）。
   */
  EsrSession(EsrSessionConfig config, extension::IInterceptPipeline& pipeline,
             environment::IEsrEnvironmentService& environment_service,
             extension::EsrController& controller);
  ~EsrSession();

  EsrSession(const EsrSession&) = delete;
  EsrSession& operator=(const EsrSession&) = delete;

  /**
   * @brief 移动构造会话。
   */
  EsrSession(EsrSession&&) noexcept;
  /**
   * @brief 移动赋值会话。
   */
  EsrSession& operator=(EsrSession&&) noexcept;

  /**
   * @brief 执行单周期并返回输出帧。
   * @param[in] input 当前周期输入。
   * @return 当前周期输出帧。
   */
  output::EsrOutputFrame Step(const session::EsrCycleInput& input);

  /**
   * @brief 执行单周期并返回聚合结果。
   * @param[in] input 当前周期输入。
   * @return 当前周期聚合结果。
   */
  EsrCycleResult StepWithResult(const session::EsrCycleInput& input);

  /**
   * @brief 应用运行期可变配置补丁。
   * @param[in] patch 运行期补丁。
   */
  void ApplyRuntimeConfig(const EsrRuntimeConfigPatch& patch);

  /**
   * @brief 尝试应用运行期可变配置补丁并返回是否生效。
   * @param[in] patch 运行期补丁。
   * @return true 表示补丁被接受并已应用；false 表示未请求更新或补丁被拒绝。
   */
  bool TryApplyRuntimeConfig(const EsrRuntimeConfigPatch& patch);

  /**
   * @brief 应用运行期补丁并返回结构化结果。
   * @param[in] patch 运行期补丁。
   * @return 结构化应用结果（含状态码）。
   */
  EsrRuntimeConfigApplyResult ApplyRuntimeConfigWithResult(const EsrRuntimeConfigPatch& patch);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session

}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SESSION_H_
