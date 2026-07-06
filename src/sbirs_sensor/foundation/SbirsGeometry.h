/**
 * @file SbirsGeometry.h
 * @brief SBIRS-inspired 基础几何工具。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_GEOMETRY_H_
#define ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_GEOMETRY_H_

#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"

namespace sbirs_sensor {
namespace foundation {

double Dot(const session::SbirsVector3M& lhs, const session::SbirsVector3M& rhs);
double Norm(const session::SbirsVector3M& value);
session::SbirsVector3M Subtract(const session::SbirsVector3M& lhs,
                                const session::SbirsVector3M& rhs);
session::SbirsVector3M Unit(const session::SbirsVector3M& value);
float ComputeAzimuthDeg(const session::SbirsVector3M& los);
float ComputeElevationDeg(const session::SbirsVector3M& los);
float AngularSeparationDeg(float az_a_deg, float el_a_deg, float az_b_deg, float el_b_deg);
bool IsEarthOcculted(const session::SbirsVector3M& satellite_position_ecef_m,
                     const session::SbirsVector3M& target_position_ecef_m, double earth_radius_m);

}  // namespace foundation
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_GEOMETRY_H_
