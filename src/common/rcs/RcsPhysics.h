/**
 * @file RcsPhysics.h
 * @brief 定义共享 RCS/植被散射的首批物理近似入口（REOS 对齐命名）。
 */

#ifndef COMMON_RCS_RCS_PHYSICS_H_
#define COMMON_RCS_RCS_PHYSICS_H_

#include <cstddef>
#include <vector>

namespace oneq {
namespace common {
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
 * @param[in] radius_m 圆柱半径（单位：m）。
 * @param[in] wavenumber_k0 自由空间波数 k0。
 * @return 圆柱体 RCS（单位：m^2）。
 */
float ComputeCylinderRcs(float radius_m, float wavenumber_k0);

/**
 * @brief 圆柱体双角散射 RCS 近似（公式 4.3.22）。
 * @param[in] wavenumber_k0 自由空间波数 k0。
 * @param[in] radius_m 圆柱半径（单位：m）。
 * @param[in] psi_i_deg 入射角（单位：deg）。
 * @param[in] psi_s_deg 散射角（单位：deg）。
 * @param[in] phi_deg 方位角差（单位：deg）。
 * @return 双站圆柱体 RCS（单位：m^2）。
 */
float ComputeBistaticCylinderRcs(float wavenumber_k0, float radius_m, float psi_i_deg,
                                 float psi_s_deg, float phi_deg);

/**
 * @brief 平面散射 RCS 近似（公式 7.4.3）。
 * @param[in] wavenumber_k0 自由空间波数 k0。
 * @param[in] radius_m 平面圆盘半径（单位：m）。
 * @param[in] theta_deg 入射角（单位：deg）。
 * @return 平面 RCS（单位：m^2）。
 */
float ComputePlanarPlateRcs(float wavenumber_k0, float radius_m, float theta_deg);

/**
 * @brief REOS 对齐入口：计算叶片相位矩阵。
 * @param[in] leaf_size_m 叶片特征尺寸（单位：m）。
 * @param[in] dielectric_constant_real 介电常数实部。
 * @param[in] incidence_deg 入射角（单位：deg）。
 * @param[in] scatter_deg 散射角（单位：deg）。
 * @return 2x2 叶片相位矩阵。
 */
LeafPhaseMatrices compute_leaf_phase_matrices(float leaf_size_m, float dielectric_constant_real,
                                              float incidence_deg, float scatter_deg);

/**
 * @brief 初始化树体散射器。
 * @param[in] config 树体散射器初始化参数（叶片数、冠层半径/高度）。
 * @return 随机生成的叶片方位/俯仰序列状态。
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

/**
 * @brief 有效 RCS 物理混合参数（标量 POD；不含模块 config 类型）。
 *
 * 字段与 AR `RcsPhysicsConfig` / RIR `RirRcsPhysicsConfig` 同形；carrier 回退
 * 策略由调用方在传入 frequency_hz 前完成。
 */
struct RcsPhysicsParams {
  bool enable_physical_rcs{true};  // 默认开启物理 RCS 估计。
  float physics_mix_ratio{1.0f};   // 物理估计占比 [0,1]；1=完全物理估计，与扫描无关。
  float cylinder_weight{0.7f};
  float min_equivalent_radius_m{0.05f};
  float max_equivalent_radius_m{5.0f};
  float min_rcs_m2{1.0e-4f};
  float max_rcs_m2{100.0f};
  float bistatic_psi_offset_deg{0.0f};
};

/**
 * @brief 视角/频率相关物理 RCS 与输入 RCS 的混合。
 *
 * @param[in] input_rcs_m2 场景输入 RCS（m²）。
 * @param[in] frequency_hz 载频（Hz）；非正时返回 input_rcs_m2。
 * @param[in] look_az_deg / look_el_deg 视线角（deg）；has_look_angles=false 时按 0。
 * @param[in] params 混合参数。`physics_mix_ratio` 是物理估计与输入 RCS 的线性混合比
 *            （0=只用输入值，1=完全物理估计），与扫描调度无关。
 * @return 混合后有效 RCS（m²）。
 */
float ComputeMixedPhysicalRcsM2(float input_rcs_m2, float frequency_hz, float look_az_deg,
                                float look_el_deg, bool has_look_angles,
                                const RcsPhysicsParams& params);

}  // namespace rcs
}  // namespace common
}  // namespace oneq

#endif  // COMMON_RCS_RCS_PHYSICS_H_
