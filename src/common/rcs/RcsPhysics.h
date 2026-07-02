/**
 * @file RcsPhysics.h
 * @brief 定义共享 RCS/植被散射的首批物理近似入口（REOS 对齐命名）。
 */

#ifndef COMMON_RCS_RCS_PHYSICS_H_
#define COMMON_RCS_RCS_PHYSICS_H_

#include <cstddef>
#include <vector>

namespace oneq {
namespace internal {
namespace rcs {

/**
 * @brief 叶片相位矩阵（2x2 简化形式）。
 */
struct LeafPhaseMatrices {
  float m11{0.0f};
  float m12{0.0f};
  float m21{0.0f};
  float m22{0.0f};
};

/**
 * @brief 树体散射器初始化参数。
 */
struct TreeScattererConfig {
  std::size_t leaf_count{0U};
  float canopy_radius_m{1.0f};
  float canopy_height_m{2.0f};
};

/**
 * @brief 树体散射器状态（叶片方位/俯仰序列）。
 */
struct TreeScattererState {
  std::vector<float> leaf_azimuth_deg;
  std::vector<float> leaf_elevation_deg;
};

/**
 * @brief 圆柱体 RCS 近似（公式 4.1.9）。
 */
float ComputeCylinderRcs(float radius_m, float wavenumber_k0);

/**
 * @brief 圆柱体双角散射 RCS 近似（公式 4.3.22）。
 */
float ComputeBistaticCylinderRcs(float wavenumber_k0, float radius_m, float psi_i_deg,
                                 float psi_s_deg, float phi_deg);

/**
 * @brief 平面散射 RCS 近似（公式 7.4.3）。
 */
float ComputePlanarPlateRcs(float wavenumber_k0, float radius_m, float theta_deg);

/**
 * @brief REOS 对齐入口：计算叶片相位矩阵。
 */
LeafPhaseMatrices compute_leaf_phase_matrices(float leaf_size_m, float dielectric_constant_real,
                                              float incidence_deg, float scatter_deg);

/**
 * @brief 初始化树体散射器。
 */
TreeScattererState InitializeTreeScatterer(const TreeScattererConfig& config);

/**
 * @brief 叶片参数方程批量计算。
 * @param[in] state 树体散射状态。
 * @param[in] va x 轴尺度参数。
 * @param[in] vb y 轴尺度参数。
 * @param[out] out_x_param 叶片参数方程 x 输出。
 * @param[out] out_y_param 叶片参数方程 y 输出。
 */
void ComputeLeavesParametricEquation(const TreeScattererState& state, float va, float vb,
                                     std::vector<float>* out_x_param,
                                     std::vector<float>* out_y_param);

/** @brief 兼容 REOS 历史命名；当前实现为标量语义实现。 */
inline float rcs_f419_xmm4r4(float radius_m, float wavenumber_k0) {
  return ComputeCylinderRcs(radius_m, wavenumber_k0);
}

/** @brief 兼容 REOS 历史命名；当前实现为标量语义实现。 */
inline float rcs_f4322_xmm4r4(float wavenumber_k0, float radius_m, float psi_i_deg,
                              float psi_s_deg, float phi_deg) {
  return ComputeBistaticCylinderRcs(wavenumber_k0, radius_m, psi_i_deg, psi_s_deg, phi_deg);
}

/** @brief 兼容 REOS 历史命名；当前实现为标量语义实现。 */
inline float RCS_f743_v128b_ps(float wavenumber_k0, float radius_m, float theta_deg) {
  return ComputePlanarPlateRcs(wavenumber_k0, radius_m, theta_deg);
}

/** @brief 兼容 REOS 历史命名；当前实现为标量语义实现。 */
inline TreeScattererState InitTreeScatterer_AVX(const TreeScattererConfig& config) {
  return InitializeTreeScatterer(config);
}

/** @brief 兼容 REOS 历史命名；当前实现为标量语义实现。 */
inline void ComputeLeavesParamEq_ymm8r4(const TreeScattererState& state, float va, float vb,
                                        std::vector<float>* out_x_param,
                                        std::vector<float>* out_y_param) {
  ComputeLeavesParametricEquation(state, va, vb, out_x_param, out_y_param);
}

}  // namespace rcs
}  // namespace internal
}  // namespace oneq

#endif  // COMMON_RCS_RCS_PHYSICS_H_
