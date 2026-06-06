/**
 * @file SarEcho.h
 * @brief SAR 内部点目标原始回波生成工具。
 */

#ifndef ONEQ_SRC_SAR_ECHO_SAR_ECHO_H_
#define ONEQ_SRC_SAR_ECHO_SAR_ECHO_H_

#include <cstddef>
#include <vector>

#include "sar/geometry/SarGeometry.h"
#include "sar/signal/SarFft.h"

namespace sar {
namespace echo {

/**
 * @brief 本地场景点目标。
 */
struct PointTarget {
  geometry::LocalPoint position_m{};
  double rcs_m2{1.0};
};

/**
 * @brief 单个点目标回波诊断。
 */
struct EchoTargetDiagnostic {
  std::size_t target_index{0U};
  double slant_range_m{0.0};
  double two_way_delay_s{0.0};
  std::size_t delay_sample_index{0U};
  double fractional_delay_samples{0.0};
  bool clipped{false};
  std::size_t clipped_samples{0U};
};

/**
 * @brief 原始回波生成配置。
 */
struct RawEchoConfig {
  double sample_rate_hz{0.0};
  double carrier_frequency_hz{0.0};
  std::size_t range_sample_count{0U};
};

/**
 * @brief 单脉冲原始回波结果。
 */
struct RawEchoResult {
  signal::ComplexVector samples{};
  std::vector<EchoTargetDiagnostic> diagnostics{};
  bool has_clipping{false};
};

bool GeneratePointTargetRawEcho(const RawEchoConfig& config,
                                const geometry::PlatformPulseState& platform,
                                const std::vector<PointTarget>& targets,
                                const signal::ComplexVector& transmit_waveform,
                                RawEchoResult* result);

}  // namespace echo
}  // namespace sar

#endif  // ONEQ_SRC_SAR_ECHO_SAR_ECHO_H_
