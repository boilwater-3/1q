/**
 * @file GeometryTransform.h
 * @brief 定义库内共享的姿态旋转、视线角解析与扫描限位几何工具。
 */

#ifndef COMMON_GEOMETRY_GEOMETRY_TRANSFORM_H_
#define COMMON_GEOMETRY_GEOMETRY_TRANSFORM_H_

#include <Eigen/Core>

namespace oneq {
namespace internal {
namespace geometry {

/**
 * @brief Vector3f 描述三维向量。
 */
struct Vector3f {
  float x{0.0f}; /**< x 分量 */
  float y{0.0f}; /**< y 分量 */
  float z{0.0f}; /**< z 分量 */
};

/**
 * @brief EulerAnglesDeg 描述欧拉角姿态（单位：度）。
 */
struct EulerAnglesDeg {
  float yaw_deg{0.0f};   /**< 偏航角（单位：deg） */
  float pitch_deg{0.0f}; /**< 俯仰角（单位：deg） */
  float roll_deg{0.0f};  /**< 横滚角（单位：deg） */
};

/**
 * @brief AzimuthElevationDeg 描述方位/俯仰角（单位：度）。
 */
struct AzimuthElevationDeg {
  float az_deg{0.0f}; /**< 方位角（单位：deg） */
  float el_deg{0.0f}; /**< 俯仰角（单位：deg） */
};

/**
 * @brief AzimuthElevationLimitsDeg 描述方位/俯仰扫描限位。
 */
struct AzimuthElevationLimitsDeg {
  float az_min_deg{-60.0f}; /**< 方位最小值（单位：deg） */
  float az_max_deg{60.0f};  /**< 方位最大值（单位：deg） */
  float el_min_deg{-30.0f}; /**< 俯仰最小值（单位：deg） */
  float el_max_deg{30.0f};  /**< 俯仰最大值（单位：deg） */
};

/**
 * @brief 对方位/俯仰角执行扫描窗口限幅。
 * @param[in] angle 待限幅角度。
 * @param[in] limits 扫描窗口。
 * @return 限幅后的方位/俯仰角。
 */
AzimuthElevationDeg ClampAzimuthElevation(const AzimuthElevationDeg& angle,
                                          const AzimuthElevationLimitsDeg& limits);

/**
 * @brief 计算机械/电子扫描窗口的交集。
 * @param[in] lhs 左侧扫描限位。
 * @param[in] rhs 右侧扫描限位。
 * @return 交集结果；无交集时退化为零宽窗口。
 */
AzimuthElevationLimitsDeg IntersectScanLimits(const AzimuthElevationLimitsDeg& lhs,
                                              const AzimuthElevationLimitsDeg& rhs);

/**
 * @brief 计算方位角差并规范化到 [-180, 180]。
 * @param[in] lhs_deg 方位角一（单位：deg）。
 * @param[in] rhs_deg 方位角二（单位：deg）。
 * @return 规范化后的方位差（单位：deg）。
 */
float ComputeAzimuthDifferenceDeg(float lhs_deg, float rhs_deg);

/**
 * @brief 将方位/俯仰角转换为单位方向向量。
 * @param[in] pointing_deg 方位/俯仰角（单位：deg）。
 * @return 单位方向向量。
 */
Eigen::Vector3f AzimuthElevationToUnitVector(const AzimuthElevationDeg& pointing_deg);

/**
 * @brief 将方向向量转换为方位/俯仰角。
 * @param[in] vector 方向向量。
 * @return 方位/俯仰角（单位：deg）。
 */
AzimuthElevationDeg UnitVectorToAzimuthElevation(const Eigen::Vector3f& vector);

/**
 * @brief 构建 Z-Y-X 顺序的欧拉角旋转矩阵。
 * @param[in] euler_deg 欧拉角（yaw/pitch/roll，单位：deg）。
 * @return 旋转矩阵。
 */
Eigen::Matrix3f BuildRotationMatrix(const EulerAnglesDeg& euler_deg);

/**
 * @brief 计算目标相对观测平台在观测机体系下的方位/俯仰角。
 * @param[in] observer_position_m 观测平台位置（单位：m）。
 * @param[in] observer_attitude_deg 观测平台姿态角（单位：deg）。
 * @param[in] target_position_m 目标位置（单位：m）。
 * @return 观测机体系下的方位/俯仰角。
 */
AzimuthElevationDeg ComputeRelativeLineOfSightAzEl(const Vector3f& observer_position_m,
                                                   const EulerAnglesDeg& observer_attitude_deg,
                                                   const Vector3f& target_position_m);

/**
 * @brief 将稳定参考系下的期望波束指向逆变换到挂架坐标系。
 * @param[in] desired_platform_pointing_deg 稳定参考系下的期望方位/俯仰角。
 * @param[in] platform_attitude_deg 当前平台姿态角。
 * @param[in] mount_angles_deg 雷达安装角。
 * @return 挂架坐标系下的波束指向。
 */
AzimuthElevationDeg ResolveStabilizedMountFramePointing(
    const AzimuthElevationDeg& desired_platform_pointing_deg,
    const EulerAnglesDeg& platform_attitude_deg, const EulerAnglesDeg& mount_angles_deg);

}  // namespace geometry
}  // namespace internal
}  // namespace oneq

#endif  // COMMON_GEOMETRY_GEOMETRY_TRANSFORM_H_
