/**
 * @file TargetFeatureUtils.h
 * @brief 定义面向外部调用方的目标构造与几何规范化工具。
 */

#ifndef AIRBORNE_RADAR_COMMON_TARGET_FEATURE_UTILS_H_
#define AIRBORNE_RADAR_COMMON_TARGET_FEATURE_UTILS_H_

#include <cstdint>

#include "1q/airborne_radar/common/model/TargetFeature.h"
#include "1q/api.hpp"
#include "1q/common/coordinate_transform.h"

namespace airborne_radar {
namespace common {
namespace utils {

/**
 * @brief 描述外部坐标转换为雷达局部坐标所需的参考系信息。
 * @note `origin_lla` 定义局部 ENU 原点，`radar_attitude_deg` 定义雷达局部坐标相对 ENU 的姿态。
 */
struct ONEQ_API RadarLocalFrameReference {
  oneq::common::LlaCoordinateDegM origin_lla{};       /**< 雷达参考原点（WGS84 LLA） */
  oneq::common::EulerAnglesDeg radar_attitude_deg{};  /**< 雷达局部坐标相对 ENU 的姿态角 */
};

/**
 * @brief 根据雷达局部笛卡尔坐标构造单个目标。
 * @param[in] external_target_id 外部目标标识符。
 * @param[in] position_x 目标局部 x 坐标（米）。
 * @param[in] position_y 目标局部 y 坐标（米）。
 * @param[in] position_z 目标局部 z 坐标（米）。
 * @param[in] velocity_x 目标速度 x 分量（m/s）。
 * @param[in] velocity_y 目标速度 y 分量（m/s）。
 * @param[in] velocity_z 目标速度 z 分量（m/s）。
 * @param[in] rcs 目标 RCS（平方米）。
 * @param[in] swerling_type 目标起伏模型编号。
 * @return 已写入位置、速度与斜距的目标特征。
 */
ONEQ_API model::TargetFeature MakeTargetFromCartesian(std::uint64_t external_target_id,
                                                      float position_x, float position_y,
                                                      float position_z, float velocity_x,
                                                      float velocity_y, float velocity_z, float rcs,
                                                      int swerling_type = 0);

/**
 * @brief 构造地面目标。
 * @note 仅用于测试代码；仿真输入不应显式区分空中/地面目标。
 * @param[in] external_target_id 外部目标标识符。
 * @param[in] position_x 目标局部 x 坐标（米）。
 * @param[in] position_y 目标局部 y 坐标（米）。
 * @param[in] rcs 目标 RCS（平方米）。
 * @param[in] velocity_x 地面目标速度 x 分量（m/s）。
 * @param[in] velocity_y 地面目标速度 y 分量（m/s）。
 * @param[in] swerling_type 目标起伏模型编号。
 * @return `z=0` 的目标特征。
 */
ONEQ_API
model::TargetFeature MakeGroundTarget(std::uint64_t external_target_id, float position_x,
                                      float position_y, float rcs = 1.0f, float velocity_x = 0.0f,
                                      float velocity_y = 0.0f, int swerling_type = 0);

/**
 * @brief 构造空中目标。
 * @note 仅用于测试代码；仿真输入不应显式区分空中/地面目标。
 * @param[in] external_target_id 外部目标标识符。
 * @param[in] position_x 目标局部 x 坐标（米）。
 * @param[in] position_y 目标局部 y 坐标（米）。
 * @param[in] position_z 目标局部 z 坐标（米）。
 * @param[in] velocity_x 目标速度 x 分量（m/s）。
 * @param[in] velocity_y 目标速度 y 分量（m/s）。
 * @param[in] velocity_z 目标速度 z 分量（m/s）。
 * @param[in] rcs 目标 RCS（平方米）。
 * @param[in] swerling_type 目标起伏模型编号。
 * @return 已写入三维位置与斜距的目标特征。
 */
ONEQ_API
model::TargetFeature MakeAirTarget(std::uint64_t external_target_id, float position_x,
                                   float position_y, float position_z, float velocity_x,
                                   float velocity_y, float velocity_z, float rcs = 1.0f,
                                   int swerling_type = 0);

/**
 * @brief 将 ECEF 坐标转换为雷达局部笛卡尔坐标。
 * @param[in] position_ecef_m 外部输入 ECEF 坐标（单位：m）。
 * @param[in] reference 雷达参考系定义。
 * @param[out] position_local_m 输出雷达局部坐标（单位：m），可为 nullptr。
 * @return 成功返回 true，输入非法或输出为空返回 false。
 */
ONEQ_API bool TryConvertEcefToRadarLocal(const oneq::common::EcefCoordinateM& position_ecef_m,
                                         const RadarLocalFrameReference& reference,
                                         oneq::common::Vector3f* position_local_m);

/**
 * @brief 将 LLA 坐标转换为雷达局部笛卡尔坐标。
 * @param[in] position_lla_deg_m 外部输入 LLA 坐标（单位：deg/m）。
 * @param[in] reference 雷达参考系定义。
 * @param[out] position_local_m 输出雷达局部坐标（单位：m），可为 nullptr。
 * @return 成功返回 true，输入非法或输出为空返回 false。
 */
ONEQ_API bool TryConvertLlaToRadarLocal(const oneq::common::LlaCoordinateDegM& position_lla_deg_m,
                                        const RadarLocalFrameReference& reference,
                                        oneq::common::Vector3f* position_local_m);

/**
 * @brief 根据 ECEF 坐标直接构造雷达局部 TargetFeature。
 * @param[in] external_target_id 外部目标标识符。
 * @param[in] position_ecef_m 外部输入 ECEF 坐标（单位：m）。
 * @param[in] reference 雷达参考系定义。
 * @param[in] velocity_x 目标局部速度 x 分量（m/s）。
 * @param[in] velocity_y 目标局部速度 y 分量（m/s）。
 * @param[in] velocity_z 目标局部速度 z 分量（m/s）。
 * @param[in] rcs 目标 RCS（平方米）。
 * @param[in] swerling_type 目标起伏模型编号。
 * @param[out] target 输出目标特征，可为 nullptr。
 * @return 成功返回 true，输入非法或输出为空返回 false。
 */
ONEQ_API bool TryMakeTargetFromEcef(std::uint64_t external_target_id,
                                    const oneq::common::EcefCoordinateM& position_ecef_m,
                                    const RadarLocalFrameReference& reference,
                                    float velocity_x, float velocity_y, float velocity_z, float rcs,
                                    int swerling_type, model::TargetFeature* target);

/**
 * @brief 根据 LLA 坐标直接构造雷达局部 TargetFeature。
 * @param[in] external_target_id 外部目标标识符。
 * @param[in] position_lla_deg_m 外部输入 LLA 坐标（单位：deg/m）。
 * @param[in] reference 雷达参考系定义。
 * @param[in] velocity_x 目标局部速度 x 分量（m/s）。
 * @param[in] velocity_y 目标局部速度 y 分量（m/s）。
 * @param[in] velocity_z 目标局部速度 z 分量（m/s）。
 * @param[in] rcs 目标 RCS（平方米）。
 * @param[in] swerling_type 目标起伏模型编号。
 * @param[out] target 输出目标特征，可为 nullptr。
 * @return 成功返回 true，输入非法或输出为空返回 false。
 */
ONEQ_API bool TryMakeTargetFromLla(std::uint64_t external_target_id,
                                   const oneq::common::LlaCoordinateDegM& position_lla_deg_m,
                                   const RadarLocalFrameReference& reference, float velocity_x,
                                   float velocity_y, float velocity_z, float rcs,
                                   int swerling_type, model::TargetFeature* target);

/**
 * @brief 规范化单个目标的几何派生量。
 * @details 当 `range_m <= 0` 且 `has_cartesian_position=true` 时，会按位置范数回填斜距；
 *          当 `range_m <= 0` 且位置全零时，不做静默修复。
 * @param[in,out] target 目标特征指针，可为 nullptr。
 */
ONEQ_API void NormalizeTargetGeometry(model::TargetFeature* target);

/**
 * @brief 批量规范化目标几何派生量。
 * @param[in,out] targets 目标特征列表指针，可为 nullptr。
 */
ONEQ_API void NormalizeTargetGeometry(model::TargetFeatureList* targets);

}  // namespace utils
}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_TARGET_FEATURE_UTILS_H_
