/**
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
