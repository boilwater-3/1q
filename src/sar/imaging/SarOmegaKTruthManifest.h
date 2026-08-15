/**
 * @file SarOmegaKTruthManifest.h
 * @brief 版本化 Omega-K 点目标真值清单的严格解析器。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_MANIFEST_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_MANIFEST_H_

#include <string>

#include "sar/imaging/SarOmegaKPointTargetAcceptance.h"

namespace sar {
namespace imaging {

/**
 * @brief 真值清单解析状态。
 */
enum class OmegaKTruthManifestStatus { kParsed = 0, kRejected = 1 };
/**
 * @brief 真值清单解析拒绝原因。
 */
enum class OmegaKTruthManifestReason {
  kNone = 0,               /**< 无 */
  kInvalidHeader = 1,      /**< 头部非法 */
  kUnsupportedVersion = 2, /**< 版本不受支持 */
  kInvalidField = 3,       /**< 字段非法 */
  kInvalidDigest = 4,      /**< 摘要非法 */
  kUnexpectedContent = 5,   /**< 内容异常 */
};

/**
 * @brief Omega-K 点目标真值清单。
 */
struct OmegaKTruthManifest {
  std::string dataset_id;        /**< 数据集 ID */
  unsigned int schema_version{0U}; /**< 清单 schema 版本 */
  bool physical_evidence{false}; /**< 是否为物理证据 */
  std::string source;            /**< 数据来源 */
  std::string acquisition_date;  /**< 采集日期 */
  std::string digest_sha256;     /**< 声明的载荷 SHA-256 摘要 */
  OmegaKPointTargetTruth truth;  /**< 点目标真值 */
  OmegaKPointTargetTolerances tolerances; /**< 验收容差 */
};

/**
 * @brief 真值清单解析结果。
 */
struct OmegaKTruthManifestParseResult {
  OmegaKTruthManifestStatus status{OmegaKTruthManifestStatus::kRejected}; /**< 解析状态 */
  OmegaKTruthManifestReason reason{OmegaKTruthManifestReason::kNone}; /**< 拒绝原因 */
  OmegaKTruthManifest manifest;   /**< 解析出的清单 */
};

/**
 * @brief 严格解析版本化 Omega-K 点目标真值清单文本。
 * @param[in] text 清单文本。
 * @return 解析结果（含状态与清单）。
 */
OmegaKTruthManifestParseResult ParseOmegaKTruthManifest(const std::string& text);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_MANIFEST_H_
