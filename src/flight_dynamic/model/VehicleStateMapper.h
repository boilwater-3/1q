#ifndef ONEQ_FLIGHT_DYNAMIC_MODEL_VEHICLESTATEMAPPER_H_
#define ONEQ_FLIGHT_DYNAMIC_MODEL_VEHICLESTATEMAPPER_H_

/**
 * @file VehicleStateMapper.h
 * @brief 定义 JSBSim 内部状态与 flight_dynamic 对外 VehicleState 之间的双向映射工具。
 *
 * @note 该头为模块内部私有，仅在 JSBSim 可用时编译；所有转换在 SI 与 JSBSim 英制
 *       单位（ft / fps / slugs / psf）之间进行。
 */

#include "1q/flight_dynamic/model/VehicleState.h"

namespace JSBSim {
class FGPropagate;
class FGAccelerations;
class FGFDMExec;
}  // namespace JSBSim

namespace oneq {
namespace flight_dynamic {
namespace config {
enum class InitialVelocityFrame;
}
namespace model {

/**
 * @brief JSBSim 内部状态与对外 VehicleState 之间的纯静态映射工具。
 *
 * 仅做单位换算与字段拷贝，不持有任何状态、不修改 JSBSim 模型（Map 为只读快照；
 * ApplyInitialConditions 通过 JSBSim 的 IC 接口写入初始条件）。
 */
class VehicleStateMapper {
 public:
  /**
   * @brief 从 JSBSim 仿真状态生成一份对外 VehicleState 快照。
   *
   * 读取 FGPropagate/FGAccelerations 的位置、速度、姿态、角速率/角加速度，以及
   * fdm_exec 属性树中的空速、质量、大气量；将英制单位换算为 SI（ft→m、fps→m/s、
   * slugs→kg、psf→Pa），角度/角速率保持 rad、rad/s。姿态取欧拉角。
   * @param[in] propagate JSBSim 位置/速度/姿态源。
   * @param[in] accelerations JSBSim 加速度/角加速度源。
   * @param[in] fdm_exec JSBSim 执行器（用于读取属性树中的空速/质量/大气等量）。
   * @param[in] sim_time_sec 当前仿真累计时间（单位：s），原样写入快照。
   * @return 填充后的 VehicleState 快照。
   */
  static VehicleState Map(const JSBSim::FGPropagate& propagate,
                          const JSBSim::FGAccelerations& accelerations, JSBSim::FGFDMExec& fdm_exec,
                          double sim_time_sec);

  /**
   * @brief 将外部运动学写入 JSBSim 初始条件（IC）。
   *
   * 位置：LLA 直注；ECEF 先转 LLA。高度 0 视为地面起点，由已加载的 reset XML 中的
   * AGL 值放置起落架；非 0 高度为空中起点或显式高度。速度：当 velocity_frame 为
   * kEcef 时在初始 LLA 处将 ECEF 速度转 NED 注入；否则按机体 UVW 注入。姿态由度转
   * 弧度后注入。
   * @param[in,out] fdm_exec JSBSim 执行器，其 IC 节点将被写入。
   * @param[in] kinematics 外部初始位置/速度/姿态。
   * @param[in] velocity_frame 初始速度注入参考系（body 或 ECEF）。
   */
  static void ApplyInitialConditions(JSBSim::FGFDMExec& fdm_exec,
                                     const coordinate::ExternalKinematics& kinematics,
                                     config::InitialVelocityFrame velocity_frame);

 private:
  static constexpr double kFtToM = 0.3048;
  static constexpr double kMToFt = 1.0 / kFtToM;
  static constexpr double kKnotsToMps = 0.514444;
  static constexpr double kSlugToKg = 14.5939;
  static constexpr double kLbfToN = 4.44822;
  static constexpr double kPsfToPa = 47.8803;
  static constexpr double kRadToDeg = 57.2957795;
  static constexpr double kDegToRad = 1.0 / kRadToDeg;
};

}  // namespace model
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_MODEL_VEHICLESTATEMAPPER_H_
