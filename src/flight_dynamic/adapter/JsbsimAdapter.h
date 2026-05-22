/**
 * @file JsbsimAdapter.h
 * @brief JSBSim FGFDMExec 生命周期管理与 Property 读写封装（内部实现，不对外暴露）。
 */

#ifndef FLIGHT_DYNAMIC_ADAPTER_JSBSIM_ADAPTER_H_
#define FLIGHT_DYNAMIC_ADAPTER_JSBSIM_ADAPTER_H_

#include <memory>
#include <string>
#include <stdexcept>

#include "1q/coordinate/types.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "1q/flight_dynamic/model/FlightDynamicInput.h"

// JSBSim 头文件（仅内部可见，不传播到公共 API）
// 使用与 JSBSim 内部一致的 include 风格（include 根为 jsbsim/src/）
#include "FGFDMExec.h"
#include "initialization/FGInitialCondition.h"
#include "models/FGPropagate.h"
#include "models/FGAccelerations.h"

namespace flight_dynamic {
namespace adapter {

/**
 * @brief JSBSim FGFDMExec 的 RAII 包装器。
 *
 * 职责：
 *   1. 管理 FGFDMExec 的创建与销毁
 *   2. 加载飞行器模型（LoadModel）
 *   3. 设置初始条件（RunIC）
 *   4. 推进积分（Run）
 *   5. 将 1Q ExternalKinematics 转换为 JSBSim 初始条件格式
 *
 * 不负责坐标系/单位转换（由 VehicleStateMapper 负责）。
 */
class JsbsimAdapter {
 public:
  /**
   * @brief 构造并初始化 JSBSim 执行器。
   *
   * @param config 会话配置。
   * @throws std::runtime_error 若 LoadModel 或 RunIC 失败。
   */
  explicit JsbsimAdapter(const config::FlightDynamicConfig& config);
  ~JsbsimAdapter();

  JsbsimAdapter(const JsbsimAdapter&) = delete;
  JsbsimAdapter& operator=(const JsbsimAdapter&) = delete;
  JsbsimAdapter(JsbsimAdapter&&) = default;
  JsbsimAdapter& operator=(JsbsimAdapter&&) = default;

  /**
   * @brief 推进一步积分。
   *
   * @param input 控制面与外部力输入（dt_sec 已在 FGFDMExec 内部设置）。
   * @return true 表示积分成功，false 表示 JSBSim 已达终止条件或发生错误。
   */
  bool Run(const model::FlightDynamicInput& input);

  /**
   * @brief 重置到新的初始运动学状态。
   *
   * @param kinematics 新初始状态（kEcef 或 kLla）。
   * @throws std::runtime_error 若 RunIC 失败。
   */
  void Reset(const oneq::coordinate::ExternalKinematics& kinematics);

  /**
   * @brief 读取 JSBSim Property 值（double）。
   */
  double GetProperty(const std::string& property_name) const;

  /**
   * @brief 写入 JSBSim Property 值（double）。
   */
  void SetProperty(const std::string& property_name, double value);

  /**
   * @brief 获取 FGPropagate 状态（位置、速度、姿态等），供 VehicleStateMapper 使用。
   */
  const JSBSim::FGPropagate& GetPropagate() const;

  /**
   * @brief 获取 FGAccelerations（加速度），供 VehicleStateMapper 使用。
   */
  const JSBSim::FGAccelerations& GetAccelerations() const;

  /**
   * @brief 获取底层 FGFDMExec，供 VehicleStateMapper 读取气动 Property。
   * @note 返回非 const 引用（GetPropertyValue 为非 const 方法）。
   */
  JSBSim::FGFDMExec& GetFdmExec() { return *fdm_exec_; }
  const JSBSim::FGFDMExec& GetFdmExec() const { return *fdm_exec_; }

 private:
  void ApplyInitialConditions(const oneq::coordinate::ExternalKinematics& kinematics);
  void ApplyControlInputs(const model::FlightDynamicInput& input);
  void ApplyExternalForces(const model::FlightDynamicInput& input);

  std::unique_ptr<JSBSim::FGFDMExec> fdm_exec_;
};

}  // namespace adapter
}  // namespace flight_dynamic

#endif  // FLIGHT_DYNAMIC_ADAPTER_JSBSIM_ADAPTER_H_
