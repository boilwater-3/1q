#include "airborne_radar/signal/association/LapjvSolver.h"

#include <algorithm>
#include <limits>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace association {

std::vector<int> LapjvSolver::Solve(const Eigen::Ref<const Eigen::MatrixXf>& cost_matrix) const {
  const Eigen::Index n = cost_matrix.rows();
  if (n <= 0 || cost_matrix.cols() != n) {
    PROJECT_LOG_ERROR("[LapjvSolver] rejected non-square or empty cost matrix: rows={} cols={}.",
                      cost_matrix.rows(), cost_matrix.cols());
    return std::vector<int>();
  }

  std::vector<float> u(static_cast<std::size_t>(n) + 1U, 0.0f);
  std::vector<float> v(static_cast<std::size_t>(n) + 1U, 0.0f);
  std::vector<int> p(static_cast<std::size_t>(n) + 1U, 0);
  std::vector<int> way(static_cast<std::size_t>(n) + 1U, 0);

  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;

    std::vector<float> minv(static_cast<std::size_t>(n) + 1U, std::numeric_limits<float>::max());
    std::vector<bool> used(static_cast<std::size_t>(n) + 1U, false);

    do {
      used[static_cast<std::size_t>(j0)] = true;
      const int i0 = p[static_cast<std::size_t>(j0)];
      float delta = std::numeric_limits<float>::max();
      int j1 = 0;

      for (int j = 1; j <= n; ++j) {
        if (used[static_cast<std::size_t>(j)]) {
          continue;
        }

        const float cur = cost_matrix(i0 - 1, j - 1) - u[static_cast<std::size_t>(i0)] -
                          v[static_cast<std::size_t>(j)];
        if (cur < minv[static_cast<std::size_t>(j)]) {
          minv[static_cast<std::size_t>(j)] = cur;
          way[static_cast<std::size_t>(j)] = j0;
        }

        if (minv[static_cast<std::size_t>(j)] < delta) {
          delta = minv[static_cast<std::size_t>(j)];
          j1 = j;
        }
      }

      for (int j = 0; j <= n; ++j) {
        if (used[static_cast<std::size_t>(j)]) {
          u[static_cast<std::size_t>(p[static_cast<std::size_t>(j)])] += delta;
          v[static_cast<std::size_t>(j)] -= delta;
        } else {
          minv[static_cast<std::size_t>(j)] -= delta;
        }
      }

      j0 = j1;
    } while (p[static_cast<std::size_t>(j0)] != 0);

    do {
      const int j1 = way[static_cast<std::size_t>(j0)];
      p[static_cast<std::size_t>(j0)] = p[static_cast<std::size_t>(j1)];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> row_to_col(static_cast<std::size_t>(n), -1);
  for (int j = 1; j <= n; ++j) {
    const int row = p[static_cast<std::size_t>(j)] - 1;
    if (row >= 0) {
      row_to_col[static_cast<std::size_t>(row)] = j - 1;
    }
  }

  return row_to_col;
}

}  // namespace association
}  // namespace signal
}  // namespace airborne_radar
