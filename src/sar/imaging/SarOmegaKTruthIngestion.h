// ============================================================================
// 【未进行设计需求，不再扩展 — DEPRECATED】
// 本文件不参与构建（见 src/sar/CMakeLists.txt 的 SAR_ENGINE_SOURCES 注释），
// 仅作为探索性参考保留。请勿新增依赖或据此实施。
// ============================================================================

﻿/**
 * @file SarOmegaKTruthIngestion.h
 * @brief Atomic Omega-K truth manifest and payload ingestion gate.
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_INGESTION_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_INGESTION_H_

#include <cstdint>
#include <string>
#include <vector>

#include "sar/imaging/SarOmegaKTruthManifest.h"

namespace sar {
namespace imaging {

enum class OmegaKTruthIngestionStatus { kSucceeded = 0, kRejected = 1 };
enum class OmegaKTruthIngestionReason {
  kNone = 0,
  kInvalidRequestId = 1,
  kManifestRejected = 2,
  kDigestRejected = 3,
  kDigestMismatch = 4,
};

struct OmegaKTruthIngestionRequest {
  std::uint64_t request_id{0U};
  std::string manifest_text;
  std::vector<std::uint8_t> payload_bytes;
};

struct OmegaKTruthIngestionResult {
  std::uint64_t request_id{0U};
  OmegaKTruthIngestionStatus status{OmegaKTruthIngestionStatus::kRejected};
  OmegaKTruthIngestionReason reason{OmegaKTruthIngestionReason::kNone};
  OmegaKTruthManifest manifest;
  std::vector<std::uint8_t> payload_bytes;
  std::string computed_sha256;
};

OmegaKTruthIngestionResult IngestOmegaKTruth(
    const OmegaKTruthIngestionRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_INGESTION_H_
