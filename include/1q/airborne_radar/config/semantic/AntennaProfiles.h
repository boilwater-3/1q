/**
 * @file AntennaProfiles.h
 * @brief 定义 semantic 天线方向图域使用的 profile 枚举。
 */

#ifndef AIRBORNE_RADAR_CONFIG_SEMANTIC_ANTENNA_PROFILES_H_
#define AIRBORNE_RADAR_CONFIG_SEMANTIC_ANTENNA_PROFILES_H_

namespace airborne_radar {
namespace config {
namespace semantic {

/**
 * @brief 方向图语义档位。
 */
enum class AntennaPatternProfile {
  kStandard = 0, /**< 标准主瓣与旁瓣行为。 */
  kLowSidelobe = 1, /**< 优先降低旁瓣泄漏。 */
  kWideCoverage = 2 /**< 优先扩大覆盖波束。 */
};

}  // namespace semantic
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SEMANTIC_ANTENNA_PROFILES_H_