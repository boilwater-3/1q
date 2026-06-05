/**
 * @file SarFft.h
 * @brief SAR 内部 FFT facade。
 */

#ifndef ONEQ_SRC_SAR_SIGNAL_SAR_FFT_H_
#define ONEQ_SRC_SAR_SIGNAL_SAR_FFT_H_

#include <complex>
#include <cstddef>
#include <vector>

namespace sar {
namespace signal {

using ComplexSample = std::complex<double>;
using ComplexVector = std::vector<ComplexSample>;

/**
 * @brief 行主序复数矩阵。
 */
struct ComplexMatrix {
  std::size_t rows{0U};
  std::size_t cols{0U};
  ComplexVector values{};

  ComplexSample& operator()(std::size_t row, std::size_t col);
  const ComplexSample& operator()(std::size_t row, std::size_t col) const;
};

/**
 * @brief 执行一维复数 FFT。
 * @note 归一化约定：forward 不归一化，inverse 除以 N。
 */
bool Fft1D(const ComplexVector& input, bool inverse, ComplexVector* output);

/**
 * @brief 对行主序矩阵逐行执行 FFT。
 */
bool FftRows(const ComplexMatrix& input, bool inverse, ComplexMatrix* output);

/**
 * @brief 对行主序矩阵逐列执行 FFT。
 */
bool FftCols(const ComplexMatrix& input, bool inverse, ComplexMatrix* output);

}  // namespace signal
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SIGNAL_SAR_FFT_H_
