/**
 * @file TrackLifecycleTypes.h
 * @brief 定义轨迹生命周期管理的输入类型。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_TYPES_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_TYPES_H_

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>

#include "1q/airborne_radar/model/JammingSemantics.h"
#include "airborne_radar/signal/tracking/GaussianTrackState.h"

namespace airborne_radar {
namespace signal {
namespace tracking {
/**
 * @brief CycleContext 描述一次处理周期的元信息。
 */
struct CycleContext {
  std::uint32_t cycle_index{0}; /**< 当前周期号。 */
  std::uint64_t batch_id{0};    /**< 当前探测批号。 */
  float dt_sec{0.0f}; /**< 当前周期时间步长（秒）。由外部平台模型提供；当值 <= 0 时，Lifecycle
                         update 会拒绝本周期输入。 */
  std::uint32_t extra_miss_tolerance{
      0}; /**< 额外失配容忍周期数。由控制平面按周期注入，用于在保护模式下延长 confirmed->lost
             的阈值。 */
};
/**
 * @brief AssociationTrackSeed 描述关联阶段使用的上一周期轨迹种子。
 */
struct AssociationTrackSeed {
  std::uint64_t association_key{0};                  /**< 关联键。 */
  bool has_position{false};                          /**< 是否具备有效的笛卡尔位置。 */
  Eigen::Vector3f position{Eigen::Vector3f::Zero()}; /**< 笛卡尔位置种子。 */
  bool has_gaussian_state{false};                    /**< 是否具备有效的高斯状态种子。 */
  GaussianTrackState gaussian_state;                 /**< 用于位置空间预测的高斯状态。 */
};
/**
 * @brief RawTrackMeasurement 描述关联后的原始量测语义。
 */
struct RawTrackMeasurement {
  std::size_t source_index{0};           /**< 原始输入目标索引，用于调试和结果回溯。 */
  std::uint64_t external_target_id{0};   /**< 外部输入原始目标标识符（0 表示未知/未提供）。 */
  std::uint64_t association_key{0};      /**< 关联键（例如关联模块输出的轨迹键）。 */
  bool matched_existing_track{false};    /**< 是否匹配到了上一周期已有轨迹。 */
  float association_cost{0.0f};          /**< 关联代价；未命中旧轨迹时为 0。 */
  bool used_position_association{false}; /**< 当前量测是否通过位置空间关联路径得到关联结果。 */
  bool used_external_association_seeds{
      false};                         /**< 当前量测关联时是否使用了外部 Lifecycle 轨迹种子。 */
  float detection_margin_db{0.0f};    /**< 当前量测探测裕量（dB）。 */
  bool has_cartesian_position{false}; /**< 当前是否具备可信的笛卡尔位置量测。 */
  Eigen::Vector3f position{Eigen::Vector3f::Zero()}; /**< 当前量测位置向量（x, y, z）。 */
  Eigen::Matrix3f measurement_covariance{
      Eigen::Matrix3f::Zero()}; /**< 笛卡尔坐标系下的量测噪声协方差矩阵 R。对于 3D 位置
                                   [x,y,z]，这是一个 3x3
                                   矩阵，由雷达方程基于信噪比推算出的物理方差经坐标转换后得到。 */
};
/**
 * @brief FilteredTrackFeature 描述轨迹滤波后的动态特征。
 */
struct FilteredTrackFeature {
  float observed_speed{0.0f};                        /**< 当前量测标量速度估计（m/s）。 */
  Eigen::Vector3f velocity{Eigen::Vector3f::Zero()}; /**< 当前量测速度向量（vx, vy, vz）。 */
  float rcs{0.0f};                                   /**< 当前量测估计 RCS。 */
  bool jamming_detected{false};                      /**< 当前量测是否检测到干扰。 */
  model::JammingSemantic dominant_jamming_semantic{
      model::JammingSemantic::kNone}; /**< 当前量测对应的主导干扰摘要类型。 */
  float jamming_severity{0.0f};               /**< 当前量测对应的残余干扰强度摘要，范围 [0, 1]。 */
};
/**
 * @brief TrackMeasurement 描述提供给 Lifecycle 的组合输入。
 * @note 该类型显式区分原始量测语义与滤波后特征语义，
 *       避免一个平面结构同时承载两类不同来源的数据。
 */
struct TrackMeasurement {
  RawTrackMeasurement raw_measurement;   /**< 关联后的原始量测部分。 */
  FilteredTrackFeature filtered_feature; /**< 轨迹滤波后的动态特征部分。 */
};

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_TYPES_H_
