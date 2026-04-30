/**
 * @file IRadarContextReader.h
 * @brief 定义仅读取雷达上下文状态的只读接口。
 */

#ifndef AIRBORNE_RADAR_EXTENSION_I_RADAR_CONTEXT_READER_H_
#define AIRBORNE_RADAR_EXTENSION_I_RADAR_CONTEXT_READER_H_

#include <cstdint>

#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace extension {

/**
 * @brief IRadarContextReader 提供只读的雷达上下文状态访问。
 */
class ONEQ_API IRadarContextReader {
 public:
  virtual ~IRadarContextReader() = default;

  /** @brief 获取当前周期场景目标列表。 */
  virtual const session::RadarSceneTargetList& GetSceneTargets() const = 0;

  /** @brief 获取当前搭载平台姿态角。 */
  virtual model::PlatformAttitudeDeg GetPlatformAttitude() const = 0;

  /** @brief 获取当前周期时间步长。 */
  virtual float GetCycleDeltaTimeSec() const = 0;

  /** @brief 获取当前输入周期号。 */
  virtual std::uint32_t GetCycleIndex() const = 0;
};

}  // namespace extension
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_EXTENSION_I_RADAR_CONTEXT_READER_H_
