// ============================================================================
// 【未进行设计需求，不再扩展 — DEPRECATED】
// 本文件不参与构建（见 src/sar/CMakeLists.txt 的 SAR_ENGINE_SOURCES 注释），
// 仅作为探索性参考保留。请勿新增依赖或据此实施。
// ============================================================================

﻿/**
 * @file SarOmegaKTruthPayloadDigest.h
 * @brief Portable SHA-256 verifier for exact Omega-K truth payload bytes.
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_PAYLOAD_DIGEST_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_PAYLOAD_DIGEST_H_

#include <cstdint>
#include <string>
#include <vector>

namespace sar {
namespace imaging {

enum class OmegaKTruthDigestStatus { kMatched = 0, kMismatched = 1, kRejected = 2 };
enum class OmegaKTruthDigestReason {
  kNone = 0,
  kInvalidRequestId = 1,
  kInvalidDeclaredDigest = 2,
};

struct OmegaKTruthDigestRequest {
  std::uint64_t request_id{0U};
  std::vector<std::uint8_t> payload_bytes;
  std::string declared_sha256;
};

struct OmegaKTruthDigestResult {
  std::uint64_t request_id{0U};
  OmegaKTruthDigestStatus status{OmegaKTruthDigestStatus::kRejected};
  OmegaKTruthDigestReason reason{OmegaKTruthDigestReason::kNone};
  std::string computed_sha256;
};

OmegaKTruthDigestResult VerifyOmegaKTruthPayloadDigest(
    const OmegaKTruthDigestRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_PAYLOAD_DIGEST_H_
