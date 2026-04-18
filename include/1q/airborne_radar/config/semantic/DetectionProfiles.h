/**
 * @file DetectionProfiles.h
 * @brief 定义 semantic 探测域使用的 profile 枚举。
 */

#ifndef AIRBORNE_RADAR_CONFIG_SEMANTIC_DETECTION_PROFILES_H_
#define AIRBORNE_RADAR_CONFIG_SEMANTIC_DETECTION_PROFILES_H_

namespace airborne_radar {
namespace config {
namespace semantic {

/**
 * @brief 目标起伏模型枚举。
 */
enum class SwerlingModel {
  kSwerling0 = 0, /**< 无起伏基线模型。 */
  kSwerling1 = 1, /**< 慢起伏目标模型 1。 */
  kSwerling2 = 2, /**< 快起伏目标模型 2。 */
  kSwerling3 = 3, /**< 慢起伏目标模型 3。 */
  kSwerling4 = 4  /**< 快起伏目标模型 4。 */
};

/**
 * @brief 平台硬件语义档位。
 */
enum class RadarHardwareProfile {
  kGenericAirborneXBand = 0, /**< 通用 X 波段机载雷达。 */
  kLongRangeHighPower = 1, /**< 长程高功率平台。 */
  kLightweightLpi = 2 /**< 轻量低截获概率平台。 */
};

/**
 * @brief 探测意图语义档位。
 */
enum class DetectionIntentProfile {
  kBalanced = 0, /**< 探测率与稳定性折中。 */
  kDetectionPriority = 1, /**< 优先提高探测敏感度。 */
  kTrackStabilityPriority = 2 /**< 优先提高后续跟踪稳定性。 */
};

/**
 * @brief RCS 融合语义档位。
 */
enum class RcsFusionProfile {
  kDisabled = 0, /**< 关闭物理 RCS 融合。 */
  kConservative = 1, /**< 使用保守融合强度。 */
  kEnhanced = 2 /**< 使用增强融合强度。 */
};

}  // namespace semantic
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SEMANTIC_DETECTION_PROFILES_H_