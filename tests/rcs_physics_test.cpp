/**
 * @file rcs_physics_test.cpp
 * @brief 验证共享 RCS 与植被散射首批入口的基础行为。
 */

#include <gtest/gtest.h>

#include <vector>

#include "common/rcs/RcsPhysics.h"

namespace oneq {
namespace internal {
namespace rcs {
namespace {

TEST(RcsPhysicsTest, CylinderRcsIncreasesWithRadius) {
  const float sigma_small = rcs_f419_xmm4r4(0.1f, 120.0f);
  const float sigma_large = rcs_f419_xmm4r4(0.3f, 120.0f);
  EXPECT_GT(sigma_small, 0.0f);
  EXPECT_GT(sigma_large, sigma_small);
}

TEST(RcsPhysicsTest, BistaticRcsDropsWithLargeScatteringAngles) {
  const float sigma_near_forward = rcs_f4322_xmm4r4(120.0f, 0.2f, 5.0f, 5.0f, 10.0f);
  const float sigma_off_axis = rcs_f4322_xmm4r4(120.0f, 0.2f, 70.0f, 70.0f, 170.0f);
  EXPECT_GT(sigma_near_forward, sigma_off_axis);
}

TEST(RcsPhysicsTest, PlanarRcsFallsAsIncidenceApproachesGrazing) {
  const float sigma_normal = RCS_f743_v128b_ps(120.0f, 0.3f, 5.0f);
  const float sigma_grazing = RCS_f743_v128b_ps(120.0f, 0.3f, 85.0f);
  EXPECT_GT(sigma_normal, sigma_grazing);
}

TEST(RcsPhysicsTest, LeafPhaseMatricesProduceSymmetricCrossTerms) {
  const LeafPhaseMatrices matrices = compute_leaf_phase_matrices(0.05f, 2.5f, 20.0f, 35.0f);
  EXPECT_GT(matrices.m11, 0.0f);
  EXPECT_GT(matrices.m22, 0.0f);
  EXPECT_FLOAT_EQ(matrices.m12, matrices.m21);
}

TEST(RcsPhysicsTest, TreeScattererInitializationAndParamEqKeepVectorSizesConsistent) {
  TreeScattererConfig config;
  config.leaf_count = 32U;
  config.canopy_radius_m = 1.2f;
  config.canopy_height_m = 3.5f;
  const TreeScattererState state = InitTreeScatterer_AVX(config);
  ASSERT_EQ(state.leaf_azimuth_deg.size(), config.leaf_count);
  ASSERT_EQ(state.leaf_elevation_deg.size(), config.leaf_count);

  std::vector<float> x_param;
  std::vector<float> y_param;
  ComputeLeavesParamEq_ymm8r4(state, 0.8f, 0.5f, &x_param, &y_param);
  EXPECT_EQ(x_param.size(), config.leaf_count);
  EXPECT_EQ(y_param.size(), config.leaf_count);
}

}  // namespace
}  // namespace rcs
}  // namespace internal
}  // namespace oneq
