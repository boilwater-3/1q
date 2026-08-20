/**
 * @file AircraftPerformanceDerivation.h
 * @brief 飞机性能推导单一源：CLmax（planform 分类）与失速速度。
 *
 * Autopilot（control_profile 推导）与 EngineManager（Vr / approach speed 推导）原先
 * 各自维护一份相同的 CLmax 表（default 1.6 / turboprop 2.0 / delta 2.5）、delta 翼
 * AR<2.5 检测与 V_stall 公式，是漂移源头。本单元将其唯一化为一份 kernel；ρ 作为
 * 入参透传：Autopilot 使用当前大气密度推导动态 TAS 包线，EngineManager 使用标准
 * 海平面密度推导起降 CAS 基准。
 *
 * 本单元为模块内部私有，不对外暴露。仅使用 C++11。
 */

#ifndef FLIGHT_DYNAMIC_AIRCRAFT_PERFORMANCE_DERIVATION_H_
#define FLIGHT_DYNAMIC_AIRCRAFT_PERFORMANCE_DERIVATION_H_

namespace oneq {
namespace flight_dynamic {

/**
 * @brief 飞机性能推导输入。
 *
 * 调用方负责读取重量、机翼几何、发动机类型、可选的 CLmax override 与 ρ。
 * helper 仅做纯计算，不接触 property tree / JSBSim。
 */
struct PerformanceDerivationInputs {
  double weight_lbs;        /**< 重量（单位：lbs） */
  double wing_area_ft2;     /**< 机翼面积（单位：ft²） */
  double wingspan_ft;       /**< 翼展（单位：ft） */
  bool is_turboprop;        /**< 是否涡桨（调用方据发动机类型判定） */
  bool has_cl_max_override; /**< cl_max_override 是否有效 */
  double cl_max_override;   /**< 可选 CLmax override（如 XML 节点）；has_cl_max_override 为真时才采用 */
};

/**
 * @brief 飞机性能推导结果。
 */
struct PerformanceDerivationResult {
  double cl_max;       /**< 选定的 CLmax（含 override 与 planform 检测） */
  double v_stall_ftps; /**< 给定 ρ 下的失速速度（单位：ft/s）；输入非法时为 0 */
};

/** @brief CLmax 默认值（planform 物理参数，非控制调参）：干净机翼 / 简单襟翼。 */
constexpr double kClMaxTakeoffDefault = 1.6;
/** @brief CLmax 默认值：高升力机翼，起飞襟翼 ≥33%。 */
constexpr double kClMaxTakeoffTurboprop = 2.0;
/** @brief CLmax 默认值：高 AoA 涡升力。 */
constexpr double kClMaxTakeoffDeltaWing = 2.5;

/** @brief delta 翼检测阈值：展弦比 = 翼展²/面积，AR < 2.5 为 delta / 低 AR 翼型。 */
constexpr double kDeltaWingArThreshold = 2.5;
/** @brief 标准海平面空气密度（slugs/ft³），用于 CAS 基准速度。 */
constexpr double kStandardSeaLevelDensitySlugsFt3 = 0.002377;

/** @brief 动态 TAS 包线推导输入。 */
struct DynamicSpeedEnvelopeInputs {
  double v_stall_mps;
  double wing_loading_lbs_ft2;
  bool is_piston;
  bool has_fbw;
  bool is_heavy;
};

/** @brief 动态 TAS 包线推导结果；输入非法时 valid=false。 */
struct DynamicSpeedEnvelopeResult {
  bool valid;
  double min_speed_mps;
  double cruise_speed_mps;
  double max_speed_mps;
  double ref_speed_mps;
};

/**
 * @brief 选定 CLmax（override 优先，否则按 planform 自动检测）。
 *
 * - 若 inputs.has_cl_max_override 为真且 inputs.cl_max_override 物理合理（> 0.5），直接采用。
 * - 否则：涡桨用 kClMaxTakeoffTurboprop；展弦比 < kDeltaWingArThreshold 用
 *   kClMaxTakeoffDeltaWing；其余用 kClMaxTakeoffDefault。
 *   （delta 检测优先于 turboprop，与既有 EngineManager 行为一致。）
 *
 * @param[in] inputs 推导输入。
 * @return 选定的 CLmax。
 */
double SelectClMax(const PerformanceDerivationInputs& inputs);

/**
 * @brief 推导 CLmax 与 V_stall = sqrt(2W / (ρ·S·CLmax))。
 *
 * CLmax 由 SelectClMax 选定；V_stall 使用调用方传入的 rho_slugs_ft3。
 * helper 不固定 ρ 来源，由调用方按 TAS/CAS 语义选择。
 *
 * @param[in] inputs 推导输入。
 * @param[in] rho_slugs_ft3 空气密度（单位：slugs/ft³），由调用方提供。
 * @return 推导结果；当输入非法（weight/area/ρ 非有限或非正）时 v_stall_ftps 为 0。
 */
PerformanceDerivationResult DeriveStallAndWingLoading(const PerformanceDerivationInputs& inputs,
                                                      double rho_slugs_ft3);

/** @brief 按当前 V_stall、翼载荷与静态机型能力推导 TAS 速度包线。 */
DynamicSpeedEnvelopeResult DeriveDynamicSpeedEnvelope(
    const DynamicSpeedEnvelopeInputs& inputs);

}  // namespace flight_dynamic
}  // namespace oneq

#endif  // FLIGHT_DYNAMIC_AIRCRAFT_PERFORMANCE_DERIVATION_H_
