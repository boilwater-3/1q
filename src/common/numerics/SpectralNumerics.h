/**
 * @file SpectralNumerics.h
 * @brief 频谱与线性代数基础数值工具。
 */

#ifndef ONEQ_SRC_COMMON_NUMERICS_SPECTRAL_NUMERICS_H_
#define ONEQ_SRC_COMMON_NUMERICS_SPECTRAL_NUMERICS_H_

#include <cstddef>
#include <complex>
#include <vector>

#include <Eigen/Core>

namespace oneq {
namespace common {
namespace numerics {

/**
 * @brief 实数 FFT 计划。
 */
struct RfftPlan {
  std::size_t fft_length{0U};
};

/**
 * @brief 初始化实数 FFT 计划。
 * @param[in] fft_length FFT 点数。
 * @param[out] plan 输出计划。
 * @return 初始化成功返回 true。
 */
bool RFFTI(std::size_t fft_length, RfftPlan* plan);

/**
 * @brief 实序列前向变换。
 * @param[in] plan FFT 计划。
 * @param[in] time_domain 时域实序列。
 * @param[out] spectrum 复频谱输出，长度与 fft_length 一致。
 * @return 成功返回 true。
 */
bool RFFTF(const RfftPlan& plan, const std::vector<double>& time_domain,
           std::vector<std::complex<double>>* spectrum);

/**
 * @brief 频域逆变换到实时域。
 * @param[in] plan FFT 计划。
 * @param[in] spectrum 复频谱输入。
 * @param[out] time_domain 输出时域实序列。
 * @return 成功返回 true。
 */
bool RFFTB(const RfftPlan& plan, const std::vector<std::complex<double>>& spectrum,
           std::vector<double>* time_domain);

/**
 * @brief 一维复数 FFT。
 * @param[in] input 输入复序列。
 * @param[in] inverse true 表示逆变换。
 * @return 复频域（或逆变换时域）序列。
 */
std::vector<std::complex<double>> ZFFT1D(const std::vector<std::complex<double>>& input,
                                         bool inverse);

/**
 * @brief 最小二乘求解 `argmin ||A x - b||_2`。
 * @param[in] matrix_a 系数矩阵 A。
 * @param[in] vector_b 观测向量 b。
 * @param[out] solution_x 解向量 x。
 * @return 成功返回 true。
 */
bool lstsqs(const Eigen::MatrixXd& matrix_a, const Eigen::VectorXd& vector_b,
            Eigen::VectorXd* solution_x);

/**
 * @brief 估计输入序列功率谱。
 * @param[in] samples 输入实序列。
 * @param[in] fft_length FFT 点数。
 * @param[out] power_spectrum 输出功率谱（前半谱，含 DC 与 Nyquist）。
 * @return 成功返回 true。
 */
bool marple_spect(const std::vector<double>& samples, std::size_t fft_length,
                  std::vector<double>* power_spectrum);

}  // namespace numerics
}  // namespace common
}  // namespace oneq

#endif  // ONEQ_SRC_COMMON_NUMERICS_SPECTRAL_NUMERICS_H_
