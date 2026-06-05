#include "sar/signal/SarFft.h"

#include <algorithm>
#include <unsupported/Eigen/FFT>

namespace sar {
namespace signal {

namespace {

bool HasValidShape(const ComplexMatrix& matrix) {
  return matrix.rows > 0U && matrix.cols > 0U && matrix.values.size() == matrix.rows * matrix.cols;
}

}  // namespace

ComplexSample& ComplexMatrix::operator()(std::size_t row, std::size_t col) {
  return values[row * cols + col];
}

const ComplexSample& ComplexMatrix::operator()(std::size_t row, std::size_t col) const {
  return values[row * cols + col];
}

bool Fft1D(const ComplexVector& input, bool inverse, ComplexVector* output) {
  if (output == nullptr || input.empty()) {
    return false;
  }

  Eigen::FFT<double> fft;
  ComplexVector transformed;
  if (inverse) {
    fft.inv(transformed, input);
  } else {
    fft.fwd(transformed, input);
  }
  *output = transformed;
  return output->size() == input.size();
}

bool FftRows(const ComplexMatrix& input, bool inverse, ComplexMatrix* output) {
  if (output == nullptr || !HasValidShape(input)) {
    return false;
  }

  output->rows = input.rows;
  output->cols = input.cols;
  output->values.assign(input.values.size(), ComplexSample(0.0, 0.0));

  ComplexVector row_input(input.cols);
  ComplexVector row_output;
  for (std::size_t row = 0; row < input.rows; ++row) {
    for (std::size_t col = 0; col < input.cols; ++col) {
      row_input[col] = input(row, col);
    }
    if (!Fft1D(row_input, inverse, &row_output) || row_output.size() != input.cols) {
      return false;
    }
    for (std::size_t col = 0; col < input.cols; ++col) {
      (*output)(row, col) = row_output[col];
    }
  }
  return true;
}

bool FftCols(const ComplexMatrix& input, bool inverse, ComplexMatrix* output) {
  if (output == nullptr || !HasValidShape(input)) {
    return false;
  }

  output->rows = input.rows;
  output->cols = input.cols;
  output->values.assign(input.values.size(), ComplexSample(0.0, 0.0));

  ComplexVector col_input(input.rows);
  ComplexVector col_output;
  for (std::size_t col = 0; col < input.cols; ++col) {
    for (std::size_t row = 0; row < input.rows; ++row) {
      col_input[row] = input(row, col);
    }
    if (!Fft1D(col_input, inverse, &col_output) || col_output.size() != input.rows) {
      return false;
    }
    for (std::size_t row = 0; row < input.rows; ++row) {
      (*output)(row, col) = col_output[row];
    }
  }
  return true;
}

}  // namespace signal
}  // namespace sar
