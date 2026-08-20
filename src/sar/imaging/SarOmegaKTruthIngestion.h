/**
 * @file SarOmegaKTruthIngestion.h
 * @brief Omega-K 真值清单与载荷的原子摄入门。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_INGESTION_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_INGESTION_H_

#include <cstdint>
#include <string>
#include <vector>

#include "sar/imaging/SarOmegaKTruthManifest.h"

namespace sar {
namespace imaging {

/**
 * @brief 真值摄入状态。
 */
enum class OmegaKTruthIngestionStatus { kSucceeded = 0, kRejected = 1 };
/**
 * @brief 真值摄入拒绝原因。
 */
enum class OmegaKTruthIngestionReason {
  kNone = 0,             /**< 无 */
  kInvalidRequestId = 1, /**< 请求 ID 非法 */
  kManifestRejected = 2, /**< 清单解析被拒绝 */
  kDigestRejected = 3,   /**< 摘要校验被拒绝 */
  kDigestMismatch = 4,    /**< 摘要不匹配 */
};

/**
 * @brief 真值摄入请求。
 */
struct OmegaKTruthIngestionRequest {
  std::uint64_t request_id{0U};            /**< 请求 ID */
  std::string manifest_text;               /**< 清单文本 */
  std::vector<std::uint8_t> payload_bytes; /**< 载荷字节 */
};

/**
 * @brief 真值摄入结果。
 */
struct OmegaKTruthIngestionResult {
  std::uint64_t request_id{0U};            /**< 关联的请求 ID */
  OmegaKTruthIngestionStatus status{OmegaKTruthIngestionStatus::kRejected}; /**< 摄入状态 */
  OmegaKTruthIngestionReason reason{OmegaKTruthIngestionReason::kNone}; /**< 拒绝原因 */
  OmegaKTruthManifest manifest;            /**< 解析出的清单 */
  std::vector<std::uint8_t> payload_bytes; /**< 摄入的载荷字节 */
  std::string computed_sha256;             /**< 计算得到的 SHA-256 摘要 */
};

/**
 * @brief 原子地摄入真值清单与载荷，校验摘要一致性。
 * @param[in] request 摄入请求。
 * @return 摄入结果（含解析清单与计算摘要）。
 */
OmegaKTruthIngestionResult IngestOmegaKTruth(
    const OmegaKTruthIngestionRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_INGESTION_H_
