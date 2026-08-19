#include "1q/precision_evaluation/DualLosFix.h"

#include <cmath>

namespace precision_evaluation {
namespace {

double Dot(const oneq::coordinate::Vector3d& lhs, const oneq::coordinate::Vector3d& rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

oneq::coordinate::Vector3d Subtract(const oneq::coordinate::EcefPositionM& lhs,
                                    const oneq::coordinate::EcefPositionM& rhs) {
  return oneq::coordinate::Vector3d(lhs.x_m - rhs.x_m, lhs.y_m - rhs.y_m, lhs.z_m - rhs.z_m);
}

}  // namespace

bool TryComputeDualLosFixM(const oneq::coordinate::EcefPositionM& origin_a,
                           const oneq::coordinate::Vector3d& direction_a,
                           const oneq::coordinate::EcefPositionM& origin_b,
                           const oneq::coordinate::Vector3d& direction_b,
                           oneq::coordinate::EcefPositionM* fix_position, double* residual_m) {
  if (fix_position == nullptr || residual_m == nullptr) {
    return false;
  }
  // 经典异面直线最近点：A(t)=Oa+t·Da、B(s)=Ob+s·Db，法向 N=Da×Db。
  // 记 a=Da·Da、b=Da·Db、c=Db·Db、w0=Oa−Ob，则
  // t=(b·e−c·d)/denom、s=(a·e−b·d)/denom，其中 d=Da·w0、e=Db·w0、denom=a·c−b²。
  const double a = Dot(direction_a, direction_a);
  const double c = Dot(direction_b, direction_b);
  if (a <= 0.0 || c <= 0.0) {
    return false;  // 零方向退化
  }
  const oneq::coordinate::Vector3d w0 = Subtract(origin_a, origin_b);
  const double b = Dot(direction_a, direction_b);
  const double d = Dot(direction_a, w0);
  const double e = Dot(direction_b, w0);
  const double denom = a * c - b * b;
  // denom = |Da×Db|²：近零 = 两线平行（含共线），无唯一最近点对。
  if (denom <= 1.0e-18 * a * c) {
    return false;
  }
  const double t = (b * e - c * d) / denom;
  const double s = (a * e - b * d) / denom;
  const oneq::coordinate::Vector3d closest_a(origin_a.x_m + t * direction_a.x,
                                             origin_a.y_m + t * direction_a.y,
                                             origin_a.z_m + t * direction_a.z);
  const oneq::coordinate::Vector3d closest_b(origin_b.x_m + s * direction_b.x,
                                             origin_b.y_m + s * direction_b.y,
                                             origin_b.z_m + s * direction_b.z);
  const double dx_m = closest_a.x - closest_b.x;
  const double dy_m = closest_a.y - closest_b.y;
  const double dz_m = closest_a.z - closest_b.z;
  *residual_m = std::sqrt(dx_m * dx_m + dy_m * dy_m + dz_m * dz_m);
  // 定位解取两最近点中点（对称最小化两线偏差）。
  fix_position->x_m = 0.5 * (closest_a.x + closest_b.x);
  fix_position->y_m = 0.5 * (closest_a.y + closest_b.y);
  fix_position->z_m = 0.5 * (closest_a.z + closest_b.z);
  return true;
}

}  // namespace precision_evaluation
