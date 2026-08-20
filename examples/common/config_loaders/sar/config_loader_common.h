#ifndef EXAMPLES_SAR_CONFIG_LOADER_COMMON_H_
#define EXAMPLES_SAR_CONFIG_LOADER_COMMON_H_

#include <string>

#include "json_reader.h"
#include "1q/sar/sar.hpp"

namespace examples {

// -- local SAR example enums (not yet part of public API) --------------------

/// SAR 成像工作模式枚举。
enum class SarWorkMode {
  kStripmap,
  kSpotlight,
  kScanSAR,
  kSlidingSpotlight,
  kDwell
};

/// 条带 SAR 子模式。
enum class StripmapModeType {
  kStandardStripmap,
  kHighResolutionStripmap,
  kWideSwathStripmap,
  kScanSARMode
};

/// 极化方式。
enum class PolarizationType {
  kSingleH,
  kSingleV,
  kDualHv,
  kDualVh,
  kQuadPol
};

/// 波束指向（左视/右视）。
enum class LookDirection {
  kLeft,
  kRight
};

/// SAR 成像算法枚举。
enum class ImagingAlgorithm {
  kRangeDoppler,
  kBackProjection,
  kOmegaK
};

/// 回退/中断模式。
enum class BackoffMode {
  kNone,
  kLinear,
  kExponential
};

// -- enum helpers ------------------------------------------------------------

inline SarWorkMode SarWorkModeFromString(const std::string& s) {
  if (s == "kStripmap") return SarWorkMode::kStripmap;
  if (s == "kSpotlight") return SarWorkMode::kSpotlight;
  if (s == "kScanSAR") return SarWorkMode::kScanSAR;
  if (s == "kSlidingSpotlight") return SarWorkMode::kSlidingSpotlight;
  if (s == "kDwell") return SarWorkMode::kDwell;
  return SarWorkMode::kStripmap;
}

inline StripmapModeType StripmapModeTypeFromString(const std::string& s) {
  if (s == "kStandardStripmap") return StripmapModeType::kStandardStripmap;
  if (s == "kHighResolutionStripmap")
    return StripmapModeType::kHighResolutionStripmap;
  if (s == "kWideSwathStripmap") return StripmapModeType::kWideSwathStripmap;
  if (s == "kScanSARMode") return StripmapModeType::kScanSARMode;
  return StripmapModeType::kStandardStripmap;
}

inline PolarizationType PolarizationTypeFromString(const std::string& s) {
  if (s == "kSingleH") return PolarizationType::kSingleH;
  if (s == "kSingleV") return PolarizationType::kSingleV;
  if (s == "kDualHv") return PolarizationType::kDualHv;
  if (s == "kDualVh") return PolarizationType::kDualVh;
  if (s == "kQuadPol") return PolarizationType::kQuadPol;
  return PolarizationType::kSingleH;
}

inline LookDirection LookDirectionFromString(const std::string& s) {
  if (s == "kLeft") return LookDirection::kLeft;
  if (s == "kRight") return LookDirection::kRight;
  return LookDirection::kLeft;
}

inline ImagingAlgorithm ImagingAlgorithmFromString(const std::string& s) {
  if (s == "kRangeDoppler") return ImagingAlgorithm::kRangeDoppler;
  if (s == "kBackProjection") return ImagingAlgorithm::kBackProjection;
  if (s == "kOmegaK") return ImagingAlgorithm::kOmegaK;
  return ImagingAlgorithm::kRangeDoppler;
}

inline BackoffMode BackoffModeFromString(const std::string& s) {
  if (s == "kNone") return BackoffMode::kNone;
  if (s == "kLinear") return BackoffMode::kLinear;
  if (s == "kExponential") return BackoffMode::kExponential;
  return BackoffMode::kNone;
}

}  // namespace examples

#endif  // EXAMPLES_SAR_CONFIG_LOADER_COMMON_H_
