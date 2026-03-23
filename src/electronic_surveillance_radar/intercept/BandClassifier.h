/**
 * @file BandClassifier.h
 * @brief 定义电子侦察频段分类工具。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_BAND_CLASSIFIER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_BAND_CLASSIFIER_H_

namespace electronic_surveillance_radar {
namespace intercept {

/**
 * @brief RadarBand 表示雷达频段分类结果。
 */
enum class RadarBand {
  kBelowP = 0, /**< 低于 P 段 */
  kP, /**< P 段 */
  kL, /**< L 段 */
  kS, /**< S 段 */
  kC, /**< C 段 */
  kX, /**< X 段 */
  kKu, /**< Ku 段 */
  kK, /**< K 段 */
  kKa, /**< Ka 段 */
  kU, /**< U 段 */
  kV, /**< V 段 */
  kW, /**< W 段 */
  kAboveW /**< 高于 W 段 */
};

/**
 * @brief BandClassifier 提供基于载频的频段分类能力。
 */
class BandClassifier final {
 public:
  /**
   * @brief 按中心频率分类雷达波段。
   * @param[in] carrier_hz 中心频率（单位：Hz）。
   * @return 频段分类结果。
   */
  static RadarBand Classify(double carrier_hz) {
    const double ghz = carrier_hz / 1.0e9;
    if (ghz < 0.23) {
      return RadarBand::kBelowP;
    }
    if (ghz < 1.0) {
      return RadarBand::kP;
    }
    if (ghz < 2.0) {
      return RadarBand::kL;
    }
    if (ghz < 4.0) {
      return RadarBand::kS;
    }
    if (ghz < 8.0) {
      return RadarBand::kC;
    }
    if (ghz < 12.0) {
      return RadarBand::kX;
    }
    if (ghz < 18.0) {
      return RadarBand::kKu;
    }
    if (ghz < 27.0) {
      return RadarBand::kK;
    }
    if (ghz < 40.0) {
      return RadarBand::kKa;
    }
    if (ghz < 60.0) {
      return RadarBand::kU;
    }
    if (ghz < 80.0) {
      return RadarBand::kV;
    }
    if (ghz < 100.0) {
      return RadarBand::kW;
    }
    return RadarBand::kAboveW;
  }

  /**
   * @brief 把频段枚举转换为稳定字符串标签。
   * @param[in] band 频段枚举。
   * @return 频段标签字符串。
   */
  static const char* ToString(RadarBand band) {
    switch (band) {
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
