/**
 * @file RirLapjvSolver.h
 * @brief 定义基于 LAPJV 最短增广路算法的全局最优指派求解器（阶段 2-T N1）。
 *
 * 副本来源：`src/airborne_radar/signal/association/LapjvSolver.*`
 * （审计基线 96de367c）；数值路径逐行一致，仅命名空间/守卫随迁。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_LAPJV_SOLVER_H_
#define REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_LAPJV_SOLVER_H_

#include <Eigen/Core>
#include <vector>

namespace remote_identification_radar {
namespace tracking {

/**
 * @brief RirLapjvSolver LAPJV 指派求解器。
 * @details 使用 LAPJV 风格的最短增广路算法求解稠密方阵代价矩阵上的线性指派问题，
 *          供关联引擎做全局最优检测-航迹分配（替代最近邻贪心）。
 */
class RirLapjvSolver final {
 public:
  /**
   * @brief 求解方阵代价矩阵上的指派关系。
   * @param cost_matrix 稠密方阵代价矩阵。
   * @return 行索引到列索引的映射；非方阵或空矩阵返回空解。
   */
  std::vector<int> Solve(const Eigen::Ref<const Eigen::MatrixXf>& cost_matrix) const;
};

}  // namespace tracking
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_TRACKING_RIR_LAPJV_SOLVER_H_
