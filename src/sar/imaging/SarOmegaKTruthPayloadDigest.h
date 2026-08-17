/**
 * @file SarOmegaKTruthPayloadDigest.h
 * @brief Omega-K 真值载荷字节的可移植 SHA-256 校验器。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_PAYLOAD_DIGEST_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_PAYLOAD_DIGEST_H_

#include <cstdint>
#include <string>
#include <vector>

namespace sar {
namespace imaging {

/**
 * @brief 摘要校验状态。
 */
enum class OmegaKTruthDigestStatus { kMatched = 0, kMismatched = 1, kRejected = 2 };
/**
 * @brief 摘要校验拒绝原因。
 */
enum class OmegaKTruthDigestReason {
  kNone = 0,                  /**< 无 */
  kInvalidRequestId = 1,      /**< 请求 ID 非法 */
  kInvalidDeclaredDigest = 2,  /**< 声明的摘要非法 */
};

/**
 * @brief 摘要校验请求。
 */
struct OmegaKTruthDigestRequest {
  std::uint64_t request_id{0U};            /**< 请求 ID */
  std::vector<std::uint8_t> payload_bytes; /**< 待校验的载荷字节 */
  std::string declared_sha256;             /**< 声明的 SHA-256 摘要 */
};

/**
 * @brief 摘要校验结果。
 */
struct OmegaKTruthDigestResult {
  std::uint64_t request_id{0U};            /**< 关联的请求 ID */
  OmegaKTruthDigestStatus status{OmegaKTruthDigestStatus::kRejected}; /**< 校验状态 */
  OmegaKTruthDigestReason reason{OmegaKTruthDigestReason::kNone}; /**< 拒绝原因 */
  std::string computed_sha256;             /**< 计算得到的 SHA-256 摘要 */
};

/**
 * @brief 校验真值载荷字节是否与声明的 SHA-256 摘要一致。
 * @param[in] request 摘要校验请求。
 * @return 校验结果（含匹配状态与计算摘要）。
 */
OmegaKTruthDigestResult VerifyOmegaKTruthPayloadDigest(
    const OmegaKTruthDigestRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_PAYLOAD_DIGEST_H_
