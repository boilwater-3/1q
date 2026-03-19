/**
 * @file TrackTypes.h
 * @brief 定义跟踪对象池与生命周期管理使用的公共轨迹类型。
 */

#ifndef AIRBORNE_RADAR_COMMON_TRACK_TYPES_H_
#define AIRBORNE_RADAR_COMMON_TRACK_TYPES_H_

#include <cstdint>

#include <Eigen/Core>

#include "1q/airborne_radar/common/JammingSemantics.h"
#include "airborne_radar/signal/tracking/GaussianTrackState.h"

namespace airborne_radar {
namespace common {

/**
 * @brief 表示轨迹对象在生命周期中的状态。
 */
enum class TrackStatus {
  /**< 候选轨迹，尚未达到确认阈值。 */
  kTentative = 0,

  /**< 已确认轨迹，可参与稳定跟踪与下游输出。 */
  kConfirmed,

  /**< 暂时丢失轨迹，等待短时重捕获。 */
  kLost,

  /**< 已回收轨迹，返回对象池可复用。 */
  kRecycled
};

/**
 * @brief 对象池内复用的轨迹重对象，承载完整生命周期信息。
 * @note 该结构同时保存生命周期状态、运动学估计和干扰摘要信息。
 */
struct TrackState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::uint64_t track_id{0}; /**< 全局唯一轨迹编号。 */
  std::uint64_t batch_id{0}; /**< 首次建轨所在发现批号。 */
  std::uint64_t external_target_id{0}; /**< 外部输入原始目标标识，0 表示未知。 */
  std::uint32_t generation{0}; /**< 复用代次，用于识别已回收对象的旧引用。 */
  TrackStatus status{TrackStatus::kTentative}; /**< 当前生命周期状态。 */
  std::uint32_t first_cycle{0}; /**< 首次创建所在周期编号。 */
  std::uint32_t last_update_cycle{0}; /**< 最近一次命中更新的周期编号。 */
  std::uint32_t miss_count{0}; /**< 连续失配计数。 */
  std::uint32_t hit_count{0}; /**< 命中累计计数。 */
  Eigen::Vector3f position{Eigen::Vector3f::Zero()}; /**< 目标位置向量 `(x, y, z)`。 */
  Eigen::Vector3f velocity{Eigen::Vector3f::Zero()}; /**< 目标速度向量 `(vx, vy, vz)`。 */
  Eigen::Vector3f acceleration{
      Eigen::Vector3f::Zero()}; /**< 目标加速度向量 `(ax, ay, az)`。 */
  float rcs{0.0f}; /**< 目标估计 RCS，单位为平方米。 */
  bool jamming_detected{false}; /**< 当前周期是否检测到与该轨迹相关的干扰。 */
  JammingSemantic dominant_jamming_semantic{
      JammingSemantic::kNone}; /**< 最近一次命中对应的主导干扰摘要语义。 */
  float jamming_severity{0.0f}; /**< 最近一次命中对应的残余干扰强度，范围 `[0, 1]`。 */
  signal::tracking::GaussianTrackState
      gaussian_state; /**< 高斯状态估计，与 Kalman 滤波器共享类型定义。 */
};

} // namespace common
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_COMMON_TRACK_TYPES_H_
