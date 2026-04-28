/**
 * @file types.h
 * @brief 定义坐标工具公共值类型。
 */

#ifndef ONEQ_COORDINATE_TYPES_H_
#define ONEQ_COORDINATE_TYPES_H_

#include "1q/api.hpp"

namespace oneq {
namespace coordinate {

/**
 * @brief WGS84 大地位置（纬度、经度、椭球高）。
 */
struct ONEQ_API LlaPositionDegM {
  double latitude_deg{0.0};  /**< 纬度（单位：deg，范围 [-90, 90]） */
  double longitude_deg{0.0}; /**< 经度（单位：deg，范围 [-180, 180]） */
  double altitude_m{0.0};    /**< 椭球高（单位：m） */
};

/**
 * @brief 地心地固位置（ECEF）。
 */
struct ONEQ_API EcefPositionM {
  double x_m{0.0}; /**< ECEF X（单位：m） */
  double y_m{0.0}; /**< ECEF Y（单位：m） */
  double z_m{0.0}; /**< ECEF Z（单位：m） */
};

/**
 * @brief 局部东-北-天位置。
 */
struct ONEQ_API EnuPositionM {
  double east_m{0.0};  /**< East 方向位置（单位：m） */
  double north_m{0.0}; /**< North 方向位置（单位：m） */
  double up_m{0.0};    /**< Up 方向位置（单位：m） */
};

/**
 * @brief 局部北-东-地位置。
 */
struct ONEQ_API NedPositionM {
  double north_m{0.0}; /**< North 方向位置（单位：m） */
  double east_m{0.0};  /**< East 方向位置（单位：m） */
  double down_m{0.0};  /**< Down 方向位置（单位：m） */
};

/**
 * @brief 局部北-天-东位置。
 */
struct ONEQ_API NuePositionM {
  double north_m{0.0}; /**< North 方向位置（单位：m） */
  double up_m{0.0};    /**< Up 方向位置（单位：m） */
  double east_m{0.0};  /**< East 方向位置（单位：m） */
};

/**
 * @brief 地心地固速度（ECEF）。
 */
struct ONEQ_API EcefVelocityMps {
  double x_mps{0.0}; /**< ECEF X 方向速度（单位：m/s） */
  double y_mps{0.0}; /**< ECEF Y 方向速度（单位：m/s） */
  double z_mps{0.0}; /**< ECEF Z 方向速度（单位：m/s） */
};

/**
 * @brief 局部东-北-天速度。
 */
struct ONEQ_API EnuVelocityMps {
  double east_mps{0.0};  /**< East 方向速度（单位：m/s） */
  double north_mps{0.0}; /**< North 方向速度（单位：m/s） */
  double up_mps{0.0};    /**< Up 方向速度（单位：m/s） */
};

/**
 * @brief 局部北-东-地速度。
 */
struct ONEQ_API NedVelocityMps {
  double north_mps{0.0}; /**< North 方向速度（单位：m/s） */
  double east_mps{0.0};  /**< East 方向速度（单位：m/s） */
  double down_mps{0.0};  /**< Down 方向速度（单位：m/s） */
};

/**
 * @brief 局部北-天-东速度。
 */
struct ONEQ_API NueVelocityMps {
  double north_mps{0.0}; /**< North 方向速度（单位：m/s） */
  double up_mps{0.0};    /**< Up 方向速度（单位：m/s） */
  double east_mps{0.0};  /**< East 方向速度（单位：m/s） */
};

/**
 * @brief 欧拉角姿态（yaw/pitch/roll，单位：deg）。
 * @note 旋转顺序为 Z-Y-X；沿用仓库现有约定，正 pitch 表示正仰角。
 */
struct ONEQ_API EulerAnglesDeg {
  double yaw_deg{0.0};   /**< 偏航角（单位：deg） */
  double pitch_deg{0.0}; /**< 俯仰角（单位：deg） */
  double roll_deg{0.0};  /**< 横滚角（单位：deg） */
};

/**
 * @brief 局部坐标系参考定义。
 */
struct ONEQ_API LocalFrameReference {
  LlaPositionDegM origin_lla{};      /**< 局部坐标参考原点 */
  EulerAnglesDeg frame_attitude_deg{}; /**< 局部坐标系相对 ENU 的姿态角 */
};

}  // namespace coordinate
}  // namespace oneq

#endif  // ONEQ_COORDINATE_TYPES_H_
