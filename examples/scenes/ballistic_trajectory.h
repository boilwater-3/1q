/**
 * @file scenes/ballistic_trajectory.h
 * @brief 弹道目标二体椭圆轨道闭式求解与解析推进（场景真值脚本用）。
 *
 * 按 rir_ballistic_scene_design_2026-08-28.md §3.1/§4.2：以地心为焦点的
 * 二体椭圆弧，过发射点与落点（WGS84 ECEF 实际半径，二者可不等）、顶高 =
 * 数据 max_alt；三约束闭式解轨道要素，Kepler 方程解析推进（无数值积分，
 * 不依赖推演层 target_inference 的 RK4，见 docs/common/contract.md 分层规则）。
 *
 * 时间基准（两段式）：场景时刻 t < t_bo 为助推段占位（保持发射点，速度
 * 0→v_bo 线性爬升，RIR 因地球遮挡不可见）；t ≥ t_bo 按椭圆弧推进，
 * t_bo = 数据顶高时刻 − 拟合发射→顶点时刻，使场景顶点时刻精确重建
 * 数据 max_alt_time_s。
 *
 * 参考系：轨道面冻结于 ECEF（地球固定系）——客户 testInfoOutput 数据以
 * 起止 LLA 给出地面航迹，站点几何（可见窗口/遮挡）也按地球固定系计算，
 * 与设计文档 §3.2 口径一致。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_BALLISTIC_TRAJECTORY_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_BALLISTIC_TRAJECTORY_H_

#include "1q/coordinate/types.h"

namespace component_attachment {
namespace app {

/// 弹道二体椭圆轨道要素与拟合统计（SolveBallisticTrajectory 的输出，
/// PropagateBallistic 的输入；四通道共享同一只读轨道）。
struct BallisticTrajectory {
  bool valid{false};                        /**< 求解成功（失败时其余字段无意义） */
  double semi_major_axis_m{0.0};            /**< 半长轴 a（m） */
  double eccentricity{0.0};                 /**< 偏心率 e（0 < e < 1） */
  double semi_latus_rectum_m{0.0};          /**< 半通径 p = a(1−e²)（m） */
  double mean_motion_rad_per_s{0.0};        /**< 平均角速度 n = sqrt(μ/a³)（rad/s） */
  double apogee_radius_m{0.0};              /**< 远地点地心距 r_a = 6371 km + max_alt（m） */
  double transfer_angle_rad{0.0};           /**< 发射→落点大圆角 D（rad） */
  double burnout_true_anomaly_rad{0.0};     /**< 发射点真近点角 ν₀（rad，(π−D, π) 区间） */
  double burnout_mean_anomaly_rad{0.0};     /**< 发射点平近点角 M₀（rad） */
  double burnout_time_s{0.0};               /**< 助推段结束时刻 t_bo（场景秒） */
  double burnout_velocity_mps{0.0};         /**< 关机点速度 v_bo = sqrt(μ(2/r₀−1/a))（m/s） */
  double fit_time_to_apogee_s{0.0};         /**< 拟合发射→顶点时刻（s；数据 max_alt_time 与之差 = t_bo） */
  double fit_flight_time_s{0.0};            /**< 拟合发射→落点时刻（s） */
  double max_alt_time_s{0.0};               /**< 数据顶高时刻（场景秒，等于 t_bo + fit_time_to_apogee_s） */
  double landing_time_s{0.0};               /**< 场景落点时刻（s，等于 t_bo + fit_flight_time_s） */
  oneq::coordinate::EcefPositionM launch_position_ecef_m{}; /**< 发射点 ECEF（助推段保持） */
  oneq::coordinate::Vector3d launch_velocity_dir{};  /**< 关机点速度方向单位矢（助推段爬升方向） */
  oneq::coordinate::Vector3d perigee_dir{};          /**< 近地点方向单位矢 P（ECEF） */
  oneq::coordinate::Vector3d in_plane_dir{};         /**< 面内横向单位矢 Q = N×P（ECEF，运动方向） */
};

/// 三约束闭式求解：r(ν₀) = |ECEF(start)|、r(ν₀+D) = |ECEF(end)|、
/// r(π) = 6371 km + max_alt。ν₀ 在 (π−D, π) 区间内 e₀(ν₀)−e₁(ν₀) 严格单调，
/// 二分必收敛；r₀ = r₁ 对称情形退化为设计文档 §3.1 表公式。失败（LLA 非法、
/// 起落点重合/对跖、顶点不高于端点、数值不收敛）返回 false 且 valid=false。
bool SolveBallisticTrajectory(const oneq::coordinate::LlaPositionDegM& start_lla,
                              const oneq::coordinate::LlaPositionDegM& end_lla,
                              double max_alt_m, double max_alt_time_s,
                              BallisticTrajectory* trajectory);

/// 按场景时刻解析求值（不累积、不漂移）：t < t_bo 返回发射点 + 线性爬升
/// 速度；t ≥ t_bo 牛顿迭代解 Kepler 方程 E − e·sinE = M₀ + n·(t−t_bo) 后
/// 换算真近点角，输出 ECEF 位置/速度。任一输出指针可为空（只取其一）。
void PropagateBallistic(const BallisticTrajectory& trajectory, double t_sec,
                        oneq::coordinate::EcefPositionM* position_ecef_m,
                        oneq::coordinate::EcefVelocityMps* velocity_ecef_mps);

}  // namespace app
}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_BALLISTIC_TRAJECTORY_H_
