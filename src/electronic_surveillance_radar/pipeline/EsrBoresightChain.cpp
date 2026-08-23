#include "electronic_surveillance_radar/pipeline/EsrBoresightChain.h"

#include <cmath>

#include "common/numerics/Constants.h"

namespace electronic_surveillance_radar {
namespace pipeline {

EsrBoresightChain::EsrBoresightChain(
    const oneq::coordinate::EulerAnglesDeg& attitude_enu_body_deg,
    double antenna_mount_az_deg, double antenna_mount_el_deg)
    // 公共链 mount 参数语义 = Body->Sensor 坐标旋转（取 R_mount⁻¹ 入链）；本模块安装
    // 偏置语义 = 光轴体方位/俯仰正偏置，两者方向相反，取反后入链（见头文件 @note）。
    : chain_(oneq::coordinate::EulerAnglesDeg(attitude_enu_body_deg.yaw_deg,
                                              attitude_enu_body_deg.pitch_deg,
                                              attitude_enu_body_deg.roll_deg),
             oneq::coordinate::EulerAnglesDeg(-antenna_mount_az_deg,
                                              -antenna_mount_el_deg, 0.0)) {}

oneq::coordinate::Vector3d EsrBoresightChain::EnuLosOfAntennaPointing(
    double antenna_az_deg, double antenna_el_deg) const {
  // 双精度构造天线系单位指向（保持历史前端正向路径精度），旋转由公共链原语承载。
  const double azimuth_rad = oneq::common::numerics::DegToRad(antenna_az_deg);
  const double elevation_rad = oneq::common::numerics::DegToRad(antenna_el_deg);
  const double horizontal = std::cos(elevation_rad);
  const oneq::coordinate::Vector3d antenna_los(
      horizontal * std::cos(azimuth_rad), horizontal * std::sin(azimuth_rad),
      std::sin(elevation_rad));
  return chain_.RotateSensorToReference(antenna_los);
}

}  // namespace pipeline
}  // namespace electronic_surveillance_radar
