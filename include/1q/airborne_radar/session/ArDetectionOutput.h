/**
 * @file ArDetectionOutput.h
 * @brief 定义 AR 工程周期发布的稳定量测/检测输出帧。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_DETECTION_OUTPUT_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_DETECTION_OUTPUT_H_

#include <array>
#include <cstdint>
#include <vector>

#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief 单周期检测成功的单目标量测记录（量测形态，分层契约规则 3）。
 * @note 坐标系为雷达局部 ENU 切平面（含平台姿态旋转，与决策 SPI 快照位置语义
 *       同帧）；量测协方差为同一坐标系下由雷达方程基于信噪比推算的 3×3 噪声
 *       矩阵 R。不携带场景真值标识/目标名（对齐 SBIRS raw detection 边界），
 *       不携带航迹/生命周期/识别语义——那些是估计/推演层产品，不属传感器输出。
 */
struct ONEQ_API ArDetectionRecord {
  float position_x_m{0.0f}; /**< 量测位置 x（单位：m；雷达局部 ENU 东向分量） */
  float position_y_m{0.0f}; /**< 量测位置 y（单位：m；雷达局部 ENU 北向分量） */
  float position_z_m{0.0f}; /**< 量测位置 z（单位：m；雷达局部 ENU 上向分量） */
  /** 3×3 量测噪声协方差 R，行主序 [xx,xy,xz, yx,yy,yz, zx,zy,zz]（单位：m²）。 */
  std::array<float, 9> measurement_covariance{};
  float detection_margin_db{0.0f}; /**< 检测裕量（单位：dB） */
};

/** @brief ArDetectionRecordList 表示量测记录集合。 */
using ArDetectionRecordList = std::vector<ArDetectionRecord>;

/** @brief 单个已完成 AR 周期发布的稳定量测输出帧。 */
struct ONEQ_API ArDetectionOutputFrame {
  std::uint32_t cycle_index{0};                /**< 当前周期号。 */
  std::uint64_t batch_id{0};                   /**< 当前批号。 */
  ArDetectionRecordList detections{};          /**< 当前周期检测成功的量测记录。 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_DETECTION_OUTPUT_H_
