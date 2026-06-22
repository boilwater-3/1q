// ============================================================================
// 【未进行设计需求，不再扩展 — DEPRECATED】
// 本文件不参与构建（见 src/sar/CMakeLists.txt 的 SAR_ENGINE_SOURCES 注释），
// 仅作为探索性参考保留。请勿新增依赖或据此实施。
// ============================================================================

﻿/**
 * @file SarOmegaKTruthManifest.h
 * @brief Strict parser for versioned Omega-K point-target truth manifests.
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_MANIFEST_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_MANIFEST_H_

#include <string>

#include "sar/imaging/SarOmegaKPointTargetAcceptance.h"

namespace sar {
namespace imaging {

enum class OmegaKTruthManifestStatus { kParsed = 0, kRejected = 1 };
enum class OmegaKTruthManifestReason {
  kNone = 0,
  kInvalidHeader = 1,
  kUnsupportedVersion = 2,
  kInvalidField = 3,
  kInvalidDigest = 4,
  kUnexpectedContent = 5,
};

struct OmegaKTruthManifest {
  std::string dataset_id;
  unsigned int schema_version{0U};
  bool physical_evidence{false};
  std::string source;
  std::string acquisition_date;
  std::string digest_sha256;
  OmegaKPointTargetTruth truth;
  OmegaKPointTargetTolerances tolerances;
};

struct OmegaKTruthManifestParseResult {
  OmegaKTruthManifestStatus status{OmegaKTruthManifestStatus::kRejected};
  OmegaKTruthManifestReason reason{OmegaKTruthManifestReason::kNone};
  OmegaKTruthManifest manifest;
};

OmegaKTruthManifestParseResult ParseOmegaKTruthManifest(const std::string& text);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_MANIFEST_H_
