/**
 * @file AssignmentSolver.h
 * @brief 定义 Signal 层数据关联使用的指派求解器抽象接口。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_ASSOCIATION_ASSIGNMENT_SOLVER_H_
#define AIRBORNE_RADAR_SIGNAL_ASSOCIATION_ASSIGNMENT_SOLVER_H_

#include <vector>

#include <Eigen/Core>

namespace airborne_radar {
namespace signal {
namespace association {
/**
 * @brief 指派求解器抽象接口。
 * @details 该接口负责在稠密代价矩阵上求解行到列的一一匹配关系。
 */
class IAssignmentSolver {
public:
/**
 * @brief 析构函数。
 */
  virtual ~IAssignmentSolver() = default;
/**
 * @brief 求解线性指派问题。
 * @param cost_matrix 输入的方阵代价矩阵。
 * @return 行索引到列索引的映射；若某行未分配则返回负值。
 */
  virtual std::vector<int> Solve(
      const Eigen::Ref<const Eigen::MatrixXf> &cost_matrix) const = 0;
};

} // namespace association
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_ASSOCIATION_ASSIGNMENT_SOLVER_H_
