/**
 * @file attitude_transform.h
 * @brief 定义不同坐标系姿态角和旋转矩阵转换工具。
 */

#ifndef ONEQ_COORDINATE_ATTITUDE_TRANSFORM_H_
#define ONEQ_COORDINATE_ATTITUDE_TRANSFORM_H_

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace oneq {
namespace coordinate {

/**
 * @brief 三维旋转矩阵。
 * @note 矩阵按行主序字段命名；公共头不暴露 Eigen。
 */
struct ONEQ_API RotationMatrix3d {
  double m00{1.0};
  double m01{0.0};
  double m02{0.0};
  double m10{0.0};
  double m11{1.0};
  double m12{0.0};
  double m20{0.0};
  double m21{0.0};
  double m22{1.0};
};

ONEQ_API bool IsFinite(const EulerAnglesDeg& attitude);
ONEQ_API bool IsFinite(const RotationMatrix3d& rotation);
ONEQ_API RotationMatrix3d BuildRotationMatrix(const EulerAnglesDeg& attitude_deg);
ONEQ_API EulerAnglesDeg ToEulerAnglesDeg(const RotationMatrix3d& rotation);
ONEQ_API RotationMatrix3d Inverse(const RotationMatrix3d& rotation);
ONEQ_API RotationMatrix3d Compose(const RotationMatrix3d& parent_to_child,
                                  const RotationMatrix3d& child_to_grandchild);
ONEQ_API EulerAnglesDeg ComposeAttitudeDeg(const EulerAnglesDeg& parent_to_child,
                                           const EulerAnglesDeg& child_to_grandchild);

}  // namespace coordinate
}  // namespace oneq

#endif  // ONEQ_COORDINATE_ATTITUDE_TRANSFORM_H_
