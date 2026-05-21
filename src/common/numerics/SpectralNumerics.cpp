#include "common/numerics/Constants.h"
#include "common/numerics/SpectralNumerics.h"

#include <Eigen/Cholesky>
#include <Eigen/QR>

#include <algorithm>
#include <cmath>
#include <limits>

namespace oneq {
namespace common {
namespace numerics {

namespace {

constexpr double kEpsilon = 1.0e-12;

std::vector<std::complex<double>> NormalizeLength(const std::vector<std::complex<double>>& input,
                                                  std::size_t length) {
  std::vector<std::complex<double>> normalized(length, std::complex<double>(0.0, 0.0));
  const std::size_t copy_count = std::min(length, input.size());
  for (std::size_t i = 0; i < copy_count; ++i) {
    normalized[i] = input[i];
  }
  return normalized;
}

}  // namespace

bool RFFTI(std::size_t fft_length, RfftPlan* plan) {
  if (plan == nullptr || fft_length == 0U) {
    return false;
  }
  plan->fft_length = fft_length;
  return true;
}

std::vector<std::complex<double>> ZFFT1D(const std::vector<std::complex<double>>& input,
                                         bool inverse) {
  const std::size_t n = input.size();
  std::vector<std::complex<double>> output(n, std::complex<double>(0.0, 0.0));
  if (n == 0U) {
    return output;
  }

  const double sign = inverse ? 1.0 : -1.0;
  for (std::size_t k = 0; k < n; ++k) {
    std::complex<double> accum(0.0, 0.0);
    for (std::size_t t = 0; t < n; ++t) {
      const double angle = sign * 2.0 * oneq::internal::numerics::constants::kPi * static_cast<double>(k * t) / static_cast<double>(n);
      const std::complex<double> kernel(std::cos(angle), std::sin(angle));
      accum += input[t] * kernel;
    }
    output[k] = inverse ? (accum / static_cast<double>(n)) : accum;
  }
  return output;
}

bool RFFTF(const RfftPlan& plan, const std::vector<double>& time_domain,
           std::vector<std::complex<double>>* spectrum) {
  if (plan.fft_length == 0U || spectrum == nullptr) {
    return false;
  }

  std::vector<std::complex<double>> complex_input(plan.fft_length, std::complex<double>(0.0, 0.0));
  const std::size_t copy_count = std::min(plan.fft_length, time_domain.size());
  for (std::size_t i = 0; i < copy_count; ++i) {
    complex_input[i] = std::complex<double>(time_domain[i], 0.0);
  }
  *spectrum = ZFFT1D(complex_input, false);
  return true;
}

bool RFFTB(const RfftPlan& plan, const std::vector<std::complex<double>>& spectrum,
           std::vector<double>* time_domain) {
  if (plan.fft_length == 0U || time_domain == nullptr) {
    return false;
  }
  const std::vector<std::complex<double>> padded_spectrum =
      NormalizeLength(spectrum, plan.fft_length);
  const std::vector<std::complex<double>> reconstructed = ZFFT1D(padded_spectrum, true);
  time_domain->assign(plan.fft_length, 0.0);
  for (std::size_t i = 0; i < plan.fft_length; ++i) {
    (*time_domain)[i] = reconstructed[i].real();
  }
  return true;
}

bool lstsqs(const Eigen::MatrixXd& matrix_a, const Eigen::VectorXd& vector_b,
            Eigen::VectorXd* solution_x) {
  if (solution_x == nullptr || matrix_a.rows() == 0 || matrix_a.cols() == 0 ||
      matrix_a.rows() != vector_b.rows()) {
    return false;
  }

  const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(matrix_a);
  if (qr.rank() == matrix_a.cols()) {
    *solution_x = qr.solve(vector_b);
    return solution_x->allFinite();
  }

  Eigen::MatrixXd normal_matrix = matrix_a.transpose() * matrix_a;
  normal_matrix.diagonal().array() += 1.0e-9;
  const Eigen::VectorXd normal_rhs = matrix_a.transpose() * vector_b;
  const Eigen::LDLT<Eigen::MatrixXd> ldlt(normal_matrix);
  if (ldlt.info() != Eigen::Success) {
    return false;
  }
  *solution_x = ldlt.solve(normal_rhs);
  return solution_x->allFinite();
}

bool marple_spect(const std::vector<double>& samples, std::size_t fft_length,
                  std::vector<double>* power_spectrum) {
  if (power_spectrum == nullptr || fft_length == 0U || samples.empty()) {
    return false;
  }

  RfftPlan plan;
  if (!RFFTI(fft_length, &plan)) {
    return false;
  }

  std::vector<std::complex<double>> spectrum;
  if (!RFFTF(plan, samples, &spectrum)) {
    return false;
  }

  const std::size_t half_spectrum_size = fft_length / 2U + 1U;
  power_spectrum->assign(half_spectrum_size, 0.0);
  const double scale = 1.0 / std::max(static_cast<double>(fft_length), 1.0);
  for (std::size_t i = 0; i < half_spectrum_size; ++i) {
    const double power = std::norm(spectrum[i]) * scale;
    (*power_spectrum)[i] = std::max(power, 0.0);
  }

  double total_power = 0.0;
  for (std::size_t i = 0; i < power_spectrum->size(); ++i) {
    total_power += (*power_spectrum)[i];
  }
  if (!std::isfinite(total_power) || total_power < kEpsilon) {
    (*power_spectrum)[0] = 1.0;
    for (std::size_t i = 1; i < power_spectrum->size(); ++i) {
      (*power_spectrum)[i] = 0.0;
    }
  }
  return true;
}

}  // namespace numerics
}  // namespace common
}  // namespace oneq
