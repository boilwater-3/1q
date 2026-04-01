/**
 * @file DistanceMetric.h
 * @brief 定义 Signal 层数据关联使用的距离度量接口与实现。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_ASSOCIATION_DISTANCE_METRIC_H_
#define AIRBORNE_RADAR_SIGNAL_ASSOCIATION_DISTANCE_METRIC_H_

#include <Eigen/Cholesky>
#include <Eigen/Core>

namespace airborne_radar {
namespace signal {
namespace association {
/**
 * @brief 距离度量抽象接口。
 */
class IDistanceMetric {
 public:
  virtual ~IDistanceMetric() = default;
  /**
   * @brief 计算预测轨迹与当前量测之间的距离代价。
   * @param predicted 历史轨迹预测特征。
   * @param measurement 当前量测特征。
   * @return 距离代价值，值越小表示越接近。
   */
  virtual float Compute(const Eigen::Vector3f& predicted,
                        const Eigen::Vector3f& measurement) const = 0;
};
/**
 * @brief 马氏距离度量实现。
 */
class MahalanobisDistanceMetric final : public IDistanceMetric {
 public:
  /**
   * @brief 构造马氏距离度量器。
   * @param speed_sigma 速度维度标准差。
   * @param rcs_sigma RCS 维度标准差。
   * @param acceleration_sigma 加速度维度标准差。
   */
  MahalanobisDistanceMetric(float speed_sigma, float rcs_sigma, float acceleration_sigma);
  /**
   * @brief 计算归一化平方距离。
   * @param predicted 历史轨迹预测特征。
   * @param measurement 当前量测特征。
   * @return 归一化平方距离。
   */
  float Compute(const Eigen::Vector3f& predicted,
                const Eigen::Vector3f& measurement) const override;

 private:
  Eigen::Array3f inverse_variances_{Eigen::Array3f::Ones()}; /**< 各维度逆方差系数。 */
};
/**
 * @brief 支持协方差注入的距离度量扩展接口。
 * @details 继承自 IDistanceMetric，增加 SetInnovationCovariance 能力。
 *          DenseCostHypothesiser 通过该接口注入逐轨迹新息协方差，
 *          无需了解底层实现类型，消除 dynamic_cast。
 */
class ICovarianceAwareDistanceMetric : public IDistanceMetric {
 public:
  virtual ~ICovarianceAwareDistanceMetric() = default;
  /**
   * @brief 更新本次轨迹计算使用的新息协方差矩阵。
   * @param S 3×3 新息协方差矩阵。
   */
  virtual void SetInnovationCovariance(const Eigen::Matrix3f& S) = 0;
};
/**
 * @brief 完整协方差矩阵马氏距离度量实现。
 * @details d² = (Δz)ᵀ S⁻¹ (Δz)，其中 S 为新息协方差矩阵。
 *          使用 LLT 分解求解，比直接求逆更稳定。
 *          对照 Stone Soup measures.Mahalanobis 的 mapping_matrix + covar 实现。
 */
class FullMahalanobisDistanceMetric final : public ICovarianceAwareDistanceMetric {
 public:
  /**
   * @brief 构造完整协方差马氏距离度量器。
   * @param innovation_covariance 3×3 新息协方差矩阵 S。
   */
  explicit FullMahalanobisDistanceMetric(const Eigen::Matrix3f& innovation_covariance);
  /**
   * @brief 从对角标准差构造（向后兼容便捷方法）。
   * @param sigma_0 维度 0 标准差。
   * @param sigma_1 维度 1 标准差。
   * @param sigma_2 维度 2 标准差。
   */
  FullMahalanobisDistanceMetric(float sigma_0, float sigma_1, float sigma_2);
  /**
   * @brief 更新新息协方差矩阵。
   * @param S 新的新息协方差。
   */
  void SetInnovationCovariance(const Eigen::Matrix3f& S) override;
  /**
   * @brief 计算完整马氏距离。
   * @param predicted 预测特征。
   * @param measurement 量测特征。
   * @return d² = Δzᵀ S⁻¹ Δz。
   */
  float Compute(const Eigen::Vector3f& predicted,
                const Eigen::Vector3f& measurement) const override;

 private:
  Eigen::LLT<Eigen::Matrix3f> llt_; /**< S 的 LLT 分解，用于高效求解 S⁻¹ Δz。 */
  bool llt_valid_{false};            /**< 当前 LLT 分解是否成功。 */
};

}  // namespace association
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_ASSOCIATION_DISTANCE_METRIC_H_
