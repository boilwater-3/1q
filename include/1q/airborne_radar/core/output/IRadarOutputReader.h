/**
 * @file IRadarOutputReader.h
 * @brief 定义供外部组件读取最新轨迹输出帧的只读接口。
 */

#ifndef AIRBORNE_RADAR_CORE_OUTPUT_I_RADAR_OUTPUT_READER_H_
#define AIRBORNE_RADAR_CORE_OUTPUT_I_RADAR_OUTPUT_READER_H_

#include "1q/airborne_radar/common/TrackOutputFrame.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace core {
namespace output {

/**
 * @brief IRadarOutputReader 抽象外部读取最新轨迹输出帧的只读能力。
 */
class ONEQ_API IRadarOutputReader {
 public:
  virtual ~IRadarOutputReader() = default;

  /**
   * @brief 判断当前是否已有可读取的最新轨迹输出帧。
   * @return 若已完成至少一次输出帧装配则返回 true。
   */
  virtual bool HasLatestTrackOutputFrame() const = 0;

  /**
   * @brief 获取最近一次已缓存的轨迹输出帧。
   * @return 最近一次运行周期产生的中性轨迹输出帧。
   */
  virtual const common::TrackOutputFrame& GetLatestTrackOutputFrame() const = 0;
};

}  // namespace output
}  // namespace core
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_OUTPUT_I_RADAR_OUTPUT_READER_H_
