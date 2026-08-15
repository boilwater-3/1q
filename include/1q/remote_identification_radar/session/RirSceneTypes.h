/**
 * @file RirSceneTypes.h
 * @brief 远程识别雷达场景实体输入类型集合。
 *
 * 场景目标输入的主头文件。识别真值字段（aspect/polarization/scatterer）随
 * 解耦从 AR `ArSceneTypes.h`（审计基线 96de367c）迁入本模块；
 * `rcs`（m²）为探测链标量，用于识别观测的 SNR 门控。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_SCENE_TYPES_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_SCENE_TYPES_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"

namespace remote_identification_radar {
namespace session {

/**
 * @brief RirAspectRcsSample 按入射方位/俯仰角离散的 RCS 真值样本。
 * @note 识别专用特征输入；RCS 以 dBsm 表达，区别于 `RirSceneTarget::rcs`（m²）。
 */
struct ONEQ_API RirAspectRcsSample {
  float aspect_az_deg{0.0f}; /**< 入射方位角（deg），任意有限值。 */
  float aspect_el_deg{0.0f}; /**< 入射俯仰角（deg），任意有限值。 */
  float rcs_dbsm{0.0f};      /**< 该视角下 RCS（dBsm），有限值。 */
};

/**
 * @brief RirPolarizationRcsSample 同一观测几何下两正交极化通道的 RCS 样本。
 */
struct ONEQ_API RirPolarizationRcsSample {
  float aspect_az_deg{0.0f};      /**< 入射方位角（deg），任意有限值。 */
  float aspect_el_deg{0.0f};      /**< 入射俯仰角（deg），任意有限值。 */
  float channel_1_rcs_dbsm{0.0f}; /**< 第一极化通道 RCS（dBsm）。 */
  float channel_2_rcs_dbsm{0.0f}; /**< 第二极化通道 RCS（dBsm）。 */
};

/**
 * @brief RirRangeRcsScatterer 距离向散射中心样本。
 * @note 一维距离像只使用 `RirSceneTarget::range_rcs_scatterers`，
 *       不允许以单个总体 RCS 代替；`phase_deg == 0` 表示非相干叠加。
 */
struct ONEQ_API RirRangeRcsScatterer {
  float range_offset_m{0.0f};       /**< 相对目标参考点的距离向位置（m）。 */
  float rcs_dbsm{0.0f};             /**< 该散射中心 RCS（dBsm）。 */
  float channel_1_rcs_dbsm{0.0f};   /**< 可选：第一极化通道散射 RCS（dBsm）。 */
  float channel_2_rcs_dbsm{0.0f};   /**< 可选：第二极化通道散射 RCS（dBsm）。 */
  float phase_deg{0.0f};            /**< 可选：相位（deg），0 表示非相干叠加。 */
  float fluctuation_std_db{0.0f};   /**< 可选：周期起伏标准差（dB），0 表示无起伏。 */
};

/**
 * @brief RirSceneTarget 描述识别雷达单周期场景目标输入。
 * @note 不含速度/加速度：运动特征由航迹供给（`RirTrackFeedEntry`）提供，
 *       场景目标只承载位置几何与特征真值。
 */
struct ONEQ_API RirSceneTarget {
  std::uint64_t external_target_id{0}; /**< 外部输入原始目标标识符（0 表示未知/未提供） */
  float position_x{0.0f};              /**< 雷达局部笛卡尔坐标 x（单位：m） */
  float position_y{0.0f};              /**< 雷达局部笛卡尔坐标 y（单位：m） */
  float position_z{0.0f};              /**< 雷达局部笛卡尔坐标 z（单位：m） */
  float rcs{0.0f};                     /**< 目标雷达散射截面积（单位：m²，SNR 门控用） */
  float range_m{0.0f};                 /**< 目标到雷达的斜距（单位：m） */

  /** 识别专用特征真值输入（默认空；空向量表示该维度不可用） */
  std::vector<RirAspectRcsSample> aspect_rcs_samples{};
  std::vector<RirPolarizationRcsSample> polarization_rcs_samples{};
  std::vector<RirRangeRcsScatterer> range_rcs_scatterers{};
};

/** @brief RirSceneTargetList 表示识别雷达场景目标输入列表。 */
using RirSceneTargetList = std::vector<RirSceneTarget>;

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_SCENE_TYPES_H_
