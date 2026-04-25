/**
 * @file IRadarControlProfileStore.h
 * @brief 定义控制真值的写入与读取接口。
 */

#ifndef AIRBORNE_RADAR_EXTENSION_I_RADAR_CONTROL_PROFILE_STORE_H_
#define AIRBORNE_RADAR_EXTENSION_I_RADAR_CONTROL_PROFILE_STORE_H_

#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace extension {

/**
 * @brief IRadarControlProfileStore 抽象控制真值的写入与读取能力。
 */
class ONEQ_API IRadarControlProfileStore {
 public:
  virtual ~IRadarControlProfileStore() = default;

  /** @brief 更新最新控制真值。 */
  virtual void UpdateRadarControlProfile(const extension::control::RadarControlProfile& profile) = 0;

  /** @brief 判断是否已保存最近一次控制真值。 */
  virtual bool HasLatestControlProfile() const = 0;

  /** @brief 获取最近一次保存的控制真值。 */
  virtual const extension::control::RadarControlProfile& GetLatestControlProfile() const = 0;
};

}  // namespace extension
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_EXTENSION_I_RADAR_CONTROL_PROFILE_STORE_H_
