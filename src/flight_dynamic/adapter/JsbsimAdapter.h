/**
 * @file JsbsimAdapter.h
 * @brief 定义对 JSBSim FGFDMExec 的薄封装，提供机型加载、积分器配置、属性读写与单步推进。
 *
 * @note 该适配器为模块内部私有头，仅在 JSBSim 可用时编译。
 */

#ifndef ONEQ_FLIGHT_DYNAMIC_ADAPTER_JSBSIM_ADAPTER_H_
#define ONEQ_FLIGHT_DYNAMIC_ADAPTER_JSBSIM_ADAPTER_H_

#include <memory>
#include <string>

#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "FGFDMExec.h"

namespace oneq {
namespace flight_dynamic {
namespace adapter {

/**
 * @brief JSBSim FGFDMExec 的薄封装，负责机型加载、初始条件注入、积分器配置与属性读写。
 *
 * 构造时加载机型、应用初始条件并尝试配平；初始化诊断记录在 InitDiagnostics 中。
 * 不可拷贝，可移动。持有 FGFDMExec 的独占所有权。
 */
class JsbsimAdapter {
 public:
  /**
   * @brief 构造适配器：加载机型、应用初始条件、配置积分器并（可选）配平。
   * @param[in] config 会话配置（机型、初始运动学、积分器等）。
   */
  explicit JsbsimAdapter(const config::FlightDynamicConfig& config);
  ~JsbsimAdapter();

  JsbsimAdapter(const JsbsimAdapter&) = delete;
  JsbsimAdapter& operator=(const JsbsimAdapter&) = delete;
  JsbsimAdapter(JsbsimAdapter&&) = default;
  JsbsimAdapter& operator=(JsbsimAdapter&&) = default;

  /**
   * @brief 推进一个仿真步长。
   * @return 推进成功返回 true；未通过初始化校验时返回 false。
   */
  bool Run();
  /**
   * @brief 应用初始条件并执行一次 RunIC。
   * @return 成功返回 true；fdm_exec_ 为空或 RunIC 失败返回 false。
   */
  bool RunIC();
  /**
   * @brief 设置仿真步长。
   * @param[in] dt_sec 步长（单位：s）。
   */
  void SetDeltaT(double dt_sec);
  /**
   * @brief 读取当前仿真步长。
   * @return 步长（单位：s）；fdm_exec_ 为空时返回 0。
   */
  double GetDeltaT() const;

  /**
   * @brief 读取 JSBSim 属性树中指定属性的数值。
   * @param[in] name 属性名。
   * @return 属性值。
   */
  double GetProperty(const std::string& name) const;
  /**
   * @brief 写入 JSBSim 属性树中指定属性的数值。
   * @param[in] name 属性名。
   * @param[in] value 待写入的值。
   */
  void SetProperty(const std::string& name, double value);
  /**
   * @brief 判断属性树中是否存在指定属性节点。
   * @param[in] name 属性名。
   * @return 存在返回 true。
   */
  bool HasProperty(const std::string& name) const;

  /** @return JSBSim FGPropagate 引用（可写）。 */
  JSBSim::FGPropagate& GetPropagate();
  /** @return JSBSim FGPropagate 引用（只读）。 */
  const JSBSim::FGPropagate& GetPropagate() const;
  /** @return JSBSim FGAccelerations 引用（可写）。 */
  JSBSim::FGAccelerations& GetAccelerations();
  /** @return JSBSim FGAccelerations 引用（只读）。 */
  const JSBSim::FGAccelerations& GetAccelerations() const;
  /** @return JSBSim FGFDMExec 引用（可写）。 */
  JSBSim::FGFDMExec& GetFdmExec();
  /** @return JSBSim FGFDMExec 引用（只读）。 */
  const JSBSim::FGFDMExec& GetFdmExec() const;

  /** @return 适配器是否就绪（fdm_exec_ 非空且 RunIC 成功）。 */
  bool IsValid() const { return fdm_exec_ != nullptr && init_diag_.run_ic_ok; }

  /**
   * @brief 构造期初始化诊断快照。
   */
  struct InitDiagnostics {
    bool model_loaded = false;            /**< 机型是否成功加载 */
    bool reset_xml_loaded = false;        /**< reset XML 是否加载 */
    bool ic_applied = false;              /**< 初始条件是否应用 */
    bool run_ic_ok = false;               /**< RunIC 是否成功 */
    bool engines_started = false;         /**< 发动机是否启动 */
    bool gear_retracted = false;          /**< 起落架是否收起 */
    bool trim_attempted = false;          /**< 是否尝试配平 */
    bool trim_succeeded = false;          /**< 配平是否成功 */
    bool trim_recovery_applied = false;   /**< 是否应用了配平失败恢复 */
    bool initialization_failed = false;   /**< 初始化是否整体失败 */
    std::string failure_reason;           /**< 失败原因描述 */
  };

  /** @return 初始化诊断（只读）。 */
  const InitDiagnostics& GetInitDiagnostics() const { return init_diag_; }
  /** @return 是否尝试过配平。 */
  bool TrimAttempted() const { return init_diag_.trim_attempted; }
  /** @return 配平是否成功。 */
  bool TrimSucceeded() const { return init_diag_.trim_succeeded; }

 private:
  bool LoadAircraft(const config::FlightDynamicConfig& config);
  void ConfigureIntegrators(const config::FlightDynamicConfig& config);

  std::unique_ptr<JSBSim::FGFDMExec> fdm_exec_;
  InitDiagnostics init_diag_;
};

}  // namespace adapter
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_ADAPTER_JSBSIM_ADAPTER_H_
