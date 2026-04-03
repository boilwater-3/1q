/**
 * @file BandClassifier.h
 * @brief 定义电子侦察频段分类工具。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_BAND_CLASSIFIER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_BAND_CLASSIFIER_H_

#include <array>
#include <cmath>
#include <cstddef>

namespace electronic_surveillance_radar {
namespace intercept {

/**
 * @brief RadarBand 表示雷达频段分类结果。
 */
enum class RadarBand {
  kInvalid = 0, /**< 输入频率非法 */
  kBelowP,      /**< 低于 P 段 */
  kP,           /**< P 段 */
  kL,           /**< L 段 */
  kS,           /**< S 段 */
  kC,           /**< C 段 */
  kX,           /**< X 段 */
  kKu,          /**< Ku 段 */
  kK,           /**< K 段 */
  kKa,          /**< Ka 段 */
  kU,           /**< U 段 */
  kV,           /**< V 段 */
  kW,           /**< W 段 */
  kAboveW       /**< 高于 W 段 */
};

/**
 * @brief BandClassifier 提供基于载频的频段分类能力。
 */
class BandClassifier final {
 public:
  /**
   * @brief FrequencyBandRange 描述单个频段区间。
   */
  struct FrequencyBandRange {
    double lower_ghz; /**< 区间下边界（单位：GHz） */
    double upper_ghz; /**< 区间上边界（单位：GHz） */
    RadarBand band;   /**< 对应频段枚举 */
  };
  using BandTable = std::array<FrequencyBandRange, 11>;

  /**
   * @brief 按中心频率分类雷达波段。
   * @param[in] carrier_hz 中心频率（单位：Hz）。
   * @return 频段分类结果。
   */
  static RadarBand Classify(double carrier_hz) {
    return Classify(carrier_hz, DefaultBandTable());
  }

  /**
   * @brief 按中心频率和给定频段表分类雷达波段。
   * @param[in] carrier_hz 中心频率（单位：Hz）。
   * @param[in] band_table 频段区间表。
   * @return 频段分类结果。
   */
  static RadarBand Classify(double carrier_hz, const BandTable& band_table) {
    if (!std::isfinite(carrier_hz) || carrier_hz <= 0.0) {
      return RadarBand::kInvalid;
    }

    const double ghz = carrier_hz / 1.0e9;
    if (ghz < band_table.front().lower_ghz) {
      return RadarBand::kBelowP;
    }
    for (std::size_t i = 0; i < band_table.size(); ++i) {
      if (ghz >= band_table[i].lower_ghz && ghz < band_table[i].upper_ghz) {
        return band_table[i].band;
      }
    }
    return RadarBand::kAboveW;
  }

  /**
   * @brief 返回默认频段区间表。
   */
  static const BandTable& DefaultBandTable() {
    static const BandTable kBandTable = {{{0.23, 1.0, RadarBand::kP},
                                          {1.0, 2.0, RadarBand::kL},
                                          {2.0, 4.0, RadarBand::kS},
                                          {4.0, 8.0, RadarBand::kC},
                                          {8.0, 12.0, RadarBand::kX},
                                          {12.0, 18.0, RadarBand::kKu},
                                          {18.0, 27.0, RadarBand::kK},
                                          {27.0, 40.0, RadarBand::kKa},
                                          {40.0, 60.0, RadarBand::kU},
                                          {60.0, 80.0, RadarBand::kV},
                                          {80.0, 100.0, RadarBand::kW}}};
    return kBandTable;
  }

  /**
   * @brief 把频段枚举转换为稳定字符串标签。
   * @param[in] band 频段枚举。
   * @return 频段标签字符串。
   */
  static const char* ToString(RadarBand band) {
    switch (band) {
      case RadarBand::kInvalid:
        return "INVALID";
      case RadarBand::kBelowP:
        return "BELOW_P";
      case RadarBand::kP:
        return "P";
      case RadarBand::kL:
        return "L";
      case RadarBand::kS:
        return "S";
      case RadarBand::kC:
        return "C";
      case RadarBand::kX:
        return "X";
      case RadarBand::kKu:
        return "Ku";
      case RadarBand::kK:
        return "K";
      case RadarBand::kKa:
        return "Ka";
      case RadarBand::kU:
        return "U";
      case RadarBand::kV:
        return "V";
      case RadarBand::kW:
        return "W";
      case RadarBand::kAboveW:
        return "ABOVE_W";
    }
    return "UNKNOWN";
  }
};

}  // namespace intercept
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_BAND_CLASSIFIER_H_
