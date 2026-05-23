/**
 * @file IRadarCommandBus.h
 * @brief 定义雷达控制指令的提交与读取接口。
 */

#ifndef ONEQ_AIRBORNE_RADAR_EXTENSION_I_RADAR_COMMAND_BUS_H_
#define ONEQ_AIRBORNE_RADAR_EXTENSION_I_RADAR_COMMAND_BUS_H_

#include <vector>

#include "1q/airborne_radar/extension/control/RadarCommand.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace extension {

/**
 * @brief IRadarCommandBus 抽象控制指令的提交与读取能力。
 */
class ONEQ_API IRadarCommandBus {
 public:
  virtual ~IRadarCommandBus() = default;

  /** @brief 提交一条控制指令。 */
  virtual void SubmitControlCommand(extension::control::RadarCommand cmd) = 0;

  /** @brief 获取当前周期已提交的控制指令列表。 */
  virtual const std::vector<extension::control::RadarCommand>& GetSubmittedCommands() const = 0;
};

}  // namespace extension
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_EXTENSION_I_RADAR_COMMAND_BUS_H_
