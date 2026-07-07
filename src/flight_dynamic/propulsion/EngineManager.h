#ifndef ONEQ_FLIGHT_DYNAMIC_PROPULSION_ENGINEMANAGER_H_
#define ONEQ_FLIGHT_DYNAMIC_PROPULSION_ENGINEMANAGER_H_

/**
 * @file EngineManager.h
 * @brief 定义发动机管理器，封装发动机类型探测、启动、油门/刹车/襟翼/起落架/混合比
 *        控制以及由机型物理量推导的起飞/进近性能参数。
 *
 * @note 该头为模块内部私有；EngineManager 按引用持有 JsbsimAdapter，不承担其所有权。
 */

#include <string>

namespace JSBSim {
class FGFDMExec;
}

namespace oneq {
namespace flight_dynamic {

namespace adapter {
class JsbsimAdapter;
}

namespace propulsion {

/**
 * @brief 发动机类型分类，由构造期探测 JSBSim 推进系统得到。
 */
enum class EngineType {
  kPiston,    /**< 活塞发动机 */
  kTurbine,   /**< 涡喷/涡扇 */
  kTurboprop, /**< 涡桨 */
  kRocket,    /**< 火箭发动机 */
  kElectric,  /**< 电动机 */
  kUnknown,   /**< 未知或元数据缺失 */
};

/**
 * @brief 发动机管理器：探测机型发动机配置，并对油门、刹车、襟翼、起落架、混合比
 *        等推进/构型控制提供统一接口；同时提供由翼载荷与失速速度推导的起飞/进近
 *        性能参数。
 *
 * 构造时一次性探测发动机类型、数量与能力（magneto/starter/mixture/WOW），并估算
 * 额定静推力缓存供后续 TWR 查询使用。
 * @note 按 adapter_ 引用持有 JsbsimAdapter，不承担所有权；调用期间须保证 adapter 存活。
 * @warning 额定推力为构造期估算值，后续仿真中不再更新。
 */
class EngineManager {
 public:
  /**
   * @brief 构造发动机管理器：探测发动机类型/数量/能力，并估算额定静推力。
   * @param[in] adapter JSBSim 适配器引用（调用方保证生命周期）。
   */
  explicit EngineManager(adapter::JsbsimAdapter& adapter);

  /** @return 探测到的发动机类型。 */
  EngineType GetType() const { return type_; }
  /** @return 发动机数量。 */
  int GetCount() const { return count_; }

  /** @return 是否存在磁电机属性（通常仅活塞机）。 */
  bool HasMagneto() const { return has_magneto_; }
  /** @return 是否存在启动机属性。 */
  bool HasStarter() const { return has_starter_; }
  /** @return 是否具备混合比控制（仅当 mixture 与 magneto 同时存在时为真）。 */
  bool HasMixture() const { return has_mixture_; }
  /** @return 属性树中是否存在承重（WOW）节点。 */
  bool HasGroundContact() const { return has_wow_; }

  /**
   * @brief 由机型物理量推导的抬轮速度 Vr（单位：kts）。
   *
   * Vr = factor × V_stall，V_stall 由重量/机翼面积/CLmax/ρ 推导；factor 按发动机类型
   * 与俯仰转动惯量（重量级代理）分类，可被 guidance/takeoff-vr-factor XML 覆盖。
   * 输入非法时返回 50 kts 回退值。
   * @return 抬轮速度（单位：kts），下限 40 kts。
   */
  double GetRotationSpeedKts() const;
  /**
   * @brief 初始爬升目标俯仰角（单位：deg），可被 guidance/climb-pitch-deg XML 覆盖。
   * @return 俯仰角（单位：deg）。
   */
  double GetClimbPitchDeg() const;
  /**
   * @brief 物理推导的进近速度回退值（单位：m/s），V_stall × 1.3（ICAO 标准余度）。
   *
   * 仅作为最低优先级回退；机动层优先使用 profile 或调用方显式给定的进近速度。
   * @return 进近速度回退值（单位：m/s），输入非法时返回 40 m/s。
   */
  double GetDefaultApproachSpeedMps() const;

  /**
   * @brief 所有发动机的额定静推力之和（单位：lbs）。
   * @return 构造期估算并缓存的额定推力；未估算时为 0。
   * @note 该值为构造期快照，仿真过程中不再更新。
   */
  double GetTotalThrustLbs() const;

  /**
   * @brief 推重比（TWR）= 额定推力 / 当前重量。
   * @return TWR；重量不可用或未估算出额定推力时返回 0.0。
   */
  double GetThrustToWeight() const;

  /**
   * @brief 启动发动机。
   *
   * 活塞机：注入全混合比 + magneto + starter；涡桨：置桨距并开 starter；
   * 涡喷/火箭/电动由适配器初始化期的 InitRunning() 启动，本方法为空操作。
   */
  void Start();

  /**
   * @brief 设置全部发动机的油门。
   * @param[in] value 归一化油门值（[0, 1]）。
   */
  void SetThrottle(double value);

  /**
   * @brief 统一设置左/右/中三组刹车。
   * @param[in] on true=刹车（1.0），false=松刹车（0.0）。
   */
  void SetBrakes(bool on);

  /**
   * @brief 设置襟翼指令。
   * @param[in] value 归一化襟翼位置（[0, 1]）。
   */
  void SetFlaps(double value);

  /**
   * @brief 收放起落架。
   * @param[in] down true=放下，false=收起。
   */
  void SetGearDown(bool down);

  /**
   * @brief 是否承重（Weight on Wheels）。
   * @return 属性树无 WOW 节点或未接地返回 false；WOW>0.5 返回 true。
   */
  bool IsWeightOnWheels() const;

  /**
   * @brief 设置燃油混合比（仅对具备混合比控制的活塞机生效）。
   * @param[in] value 归一化混合比（[0, 1]）。
   */
  void SetMixture(double value);

 private:
  void DetectType();
  void MeasureRatedThrust();
  void SetIndexedProperty(const std::string& base, int index, double value);
  double GetProperty(const std::string& name) const;

  adapter::JsbsimAdapter& adapter_;
  JSBSim::FGFDMExec& exec_;
  EngineType type_ = EngineType::kUnknown;
  int count_ = 0;
  double rated_thrust_lbs_ = 0.0;
  bool has_magneto_ = false;
  bool has_starter_ = false;
  bool has_mixture_ = false;
  bool has_wow_ = false;
};

}  // namespace propulsion
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_PROPULSION_ENGINEMANAGER_H_
