/**
 * @file IEsrEnvironmentService.h
 * @brief 定义电子侦察环境服务只读接口与快照类型。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_I_ESR_ENVIRONMENT_SERVICE_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_I_ESR_ENVIRONMENT_SERVICE_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrEnvironmentConfig.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrEnvironmentSnapshot 描述单周期环境快照。
 */
struct ONEQ_API EsrEnvironmentSnapshot {
  std::uint32_t cycle_index{0U};
  float dt_sec{0.0f};
  float propagation_loss_db{0.0f};
  float clutter_noise_w{0.0f};
  float spectrum_occupancy_ratio{0.0f}; /**< 冻结占用率；当前仅冻结与回放，检测链尚未消费（见 docs/common/open_questions.md ESR-OQ-1） */
};

}  // namespace session

namespace environment {

/**
 * @brief IEsrEnvironmentService 为电子侦察管线提供周期冻结与快照采样能力。
 */
class IEsrEnvironmentService {
 public:
  virtual ~IEsrEnvironmentService() = default;

  /**
   * @brief 冻结当前周期环境事实。
   * @param[in] cycle_index 当前周期号。
   * @param[in] dt_sec 当前周期步长。
   * @param[in] platform_altitude_m 接收平台 WGS84 绝对海拔（单位：m）。
   */
  virtual void BeginCycle(std::uint32_t cycle_index, float dt_sec,
                          float platform_altitude_m) = 0;

  /**
   * @brief 采样当前周期环境快照。
   * @return 当前周期环境快照。
   */
  virtual session::EsrEnvironmentSnapshot SampleEnvironment() const = 0;

  /**
   * @brief 更新环境模型配置。
   * @param[in] config 新的内部模型配置。
   */
  virtual void UpdateModelConfig(config::EsrEnvironmentScenarioConfig config) = 0;
};

}  // namespace environment
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_ENVIRONMENT_I_ESR_ENVIRONMENT_SERVICE_H_
