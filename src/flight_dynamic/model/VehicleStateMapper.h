/**
 * @file VehicleStateMapper.h
 * @brief JSBSim 状态 → 1Q VehicleState/ExternalKinematics 映射器（内部实现）。
 */

#ifndef FLIGHT_DYNAMIC_MODEL_VEHICLE_STATE_MAPPER_H_
#define FLIGHT_DYNAMIC_MODEL_VEHICLE_STATE_MAPPER_H_

#include "1q/coordinate/types.h"
#include "1q/flight_dynamic/model/FlightDynamicOutput.h"

// JSBSim 前置声明（仅内部可见）
namespace JSBSim {
class FGPropagate;
class FGAccelerations;
class FGFDMExec;
}  // namespace JSBSim

namespace flight_dynamic {
namespace model {

/**
 * @brief 将 JSBSim 积分结果映射到 1Q 坐标系/单位。
 *
 * 坐标系转换链：
 *   JSBSim ECEF     → 1Q ECEF     (直接映射，FGLocation → EcefPositionM)
 *   JSBSim NED 速度 → 1Q ECEF 速度 (NED→ENU→ECEF)
 *   JSBSim NED 四元数 → 1Q ENU 欧拉角 (qAttitudeLocal→NED欧拉角→ToEnuAttitude)
 *   单位：ft→m, kts→m/s, ft/s→m/s, slug·ft²→N/A（角速度 rad/s 直接映射）
 */
class VehicleStateMapper {
 public:
  VehicleStateMapper() = default;

  /**
   * @brief 从 JSBSim FGFDMExec 完整状态生成 FlightDynamicOutput。
   *
   * @param propagate FGPropagate — 位置/速度/姿态/角速度积分状态
   * @param accelerations FGAccelerations — 加速度
   * @param fdm_exec FGFDMExec — 用于读取 Property（空速、马赫数等）
   * @return 转换后的输出（SI 单位，1Q 坐标系）
   */
  FlightDynamicOutput Map(const JSBSim::FGPropagate& propagate,
                          const JSBSim::FGAccelerations& accelerations,
                          JSBSim::FGFDMExec& fdm_exec) const;

 private:
  // ---- 各子系统映射 ----
  void MapPosition(const JSBSim::FGPropagate& propagate,
                   FlightDynamicOutput& out) const;

  void MapVelocity(const JSBSim::FGPropagate& propagate,
                   const oneq::coordinate::LlaPositionDegM& lla,
                   FlightDynamicOutput& out) const;

  void MapAttitude(const JSBSim::FGPropagate& propagate,
                   FlightDynamicOutput& out) const;

  void MapAngularVelocity(const JSBSim::FGPropagate& propagate,
                          FlightDynamicOutput& out) const;

  void MapAcceleration(const JSBSim::FGAccelerations& accelerations,
                       FlightDynamicOutput& out) const;

  void MapAeroParams(JSBSim::FGFDMExec& fdm_exec,
                     FlightDynamicOutput& out) const;
};

}  // namespace model
}  // namespace flight_dynamic

#endif  // FLIGHT_DYNAMIC_MODEL_VEHICLE_STATE_MAPPER_H_
