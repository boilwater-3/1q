/**
 * @file RirTrackFeedTypes.h
 * @brief 远程识别雷达航迹供给类型集合。
 *
 * 识别模块消费外部雷达（如机载雷达 AR）已确认航迹的跨模块供给 DTO。
 * 字段集合为 AR `TrackStateSnapshot`（审计基线 96de367c）中识别链路实际
 * 消费子集的镜像（位置/速度/加速度/不确定度/威胁类别），映射语义
 * 以 `MotionFeatureExtractor` 实际读取字段为准（阶段 1 步骤 3 逐字段核对）。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_TRACK_FEED_TYPES_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_TRACK_FEED_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"

namespace remote_identification_radar {
namespace session {

/**
 * @brief RirTrackFeedStatus 航迹供给生命周期状态（镜像 AR TrackStatus 语义）。
 */
enum class ONEQ_API RirTrackFeedStatus {
  kTentative = 0, /**< 候选轨迹 */
  kConfirmed,     /**< 已确认轨迹 */
  kLost           /**< 丢失轨迹 */
};

/**
 * @brief RirTrackFeedEntry 单条航迹供给快照。
 */
struct ONEQ_API RirTrackFeedEntry {
  std::uint64_t association_key{0};    /**< 关联键（跨周期稳定标识） */
  std::uint64_t external_target_id{0}; /**< 外部输入原始目标标识符（0 表示未知/未提供） */
  RirTrackFeedStatus status{RirTrackFeedStatus::kTentative}; /**< 航迹生命周期状态 */

  float position_x{0.0f}; /**< 雷达局部笛卡尔坐标 x（m；平台 ENU 切平面东向分量，含平台姿态旋转） */
  float position_y{0.0f}; /**< 雷达局部笛卡尔坐标 y（m；平台 ENU 切平面北向分量，含平台姿态旋转） */
  float position_z{0.0f}; /**< 雷达局部笛卡尔坐标 z（m；平台 ENU 切平面上向分量，含平台姿态旋转） */

  float velocity_x{0.0f}; /**< 目标速度向量 x 分量（m/s） */
  float velocity_y{0.0f}; /**< 目标速度向量 y 分量（m/s） */
  float velocity_z{0.0f}; /**< 目标速度向量 z 分量（m/s） */

  float speed{0.0f}; /**< 速度模长（m/s） */

  float acceleration_x{0.0f}; /**< 目标加速度向量 x 分量（m/s²） */
  float acceleration_y{0.0f}; /**< 目标加速度向量 y 分量（m/s²） */
  float acceleration_z{0.0f}; /**< 目标加速度向量 z 分量（m/s²） */

  float acceleration{0.0f}; /**< 加速度模长（m/s²） */

  /**
   * @brief 航迹估计不确定度（m²），预测协方差 P 的 position 分块迹；
   *        识别运动质量因子的本源信号。
   */
  float estimation_uncertainty_trace{0.0f};

  /** @brief 供给方决策层填充的目标分类类型（"UNKNOWN"/"LOW_THREAT_TARGET"/
   *        "HIGH_THREAT_FIGHTER" 等），供识别驻留候选选择消费。 */
  std::string target_type{"UNKNOWN"};
  /** @brief 目标分类置信度，范围 [0, 1]。 */
  float target_probability{0.0f};
};

/** @brief RirTrackFeed 表示单周期航迹供给集合。 */
using RirTrackFeed = std::vector<RirTrackFeedEntry>;

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_TRACK_FEED_TYPES_H_
