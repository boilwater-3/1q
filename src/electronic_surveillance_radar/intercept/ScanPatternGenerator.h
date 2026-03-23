/**
 * @file ScanPatternGenerator.h
 * @brief 定义电子侦察波束扫描排布生成器。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_SCAN_PATTERN_GENERATOR_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_SCAN_PATTERN_GENERATOR_H_

#include <algorithm>
#include <cmath>
#include <vector>

namespace electronic_surveillance_radar {
namespace intercept {

/**
 * @brief BeamPointingDeg 描述单个波束指向。
 */
struct BeamPointingDeg {
  float az_deg{0.0f}; /**< 波束中心方位（单位：deg） */
  float el_deg{0.0f}; /**< 波束中心俯仰（单位：deg） */
};

/**
 * @brief ScanPatternConfig 描述扫描排布配置。
 */
struct ScanPatternConfig {
  float start_az_deg{-60.0f}; /**< 扫描起始方位（单位：deg） */
  float end_az_deg{60.0f}; /**< 扫描结束方位（单位：deg） */
  float start_el_deg{-10.0f}; /**< 扫描起始俯仰（单位：deg） */
  float end_el_deg{10.0f}; /**< 扫描结束俯仰（单位：deg） */
  float az_step_deg{5.0f}; /**< 方位步进（单位：deg） */
  float el_step_deg{5.0f}; /**< 俯仰步进（单位：deg） */
  int start_pos{0}; /**< 扫描起点（0: 左上, 1: 右上, 2: 右下, 3: 左下） */
  int sequence{0}; /**< 扫描顺序（0: 方位快扫, 1: 俯仰快扫） */
};

/**
 * @brief ScanPatternGenerator 负责生成扫描波束序列。
 */
class ScanPatternGenerator final {
 public:
  /**
   * @brief 生成波束扫描序列。
   * @param[in] config 扫描配置。
   * @return 按扫描顺序排列的波束指向序列。
   */
  static std::vector<BeamPointingDeg> Generate(const ScanPatternConfig& config) {
    const float az_step = config.az_step_deg > 0.0f ? config.az_step_deg : 1.0f;
    const float el_step = config.el_step_deg > 0.0f ? config.el_step_deg : 1.0f;

    float az_end = config.end_az_deg;
    if (az_end <= config.start_az_deg) {
      az_end += 360.0f;
    }

    std::vector<float> az_values;
    for (float az = config.start_az_deg; az <= az_end + 0.5f * az_step;
         az += az_step) {
      az_values.push_back(NormalizeAzimuthDeg(az));
    }
    if (az_values.empty()) {
      az_values.push_back(NormalizeAzimuthDeg(config.start_az_deg));
    }

    std::vector<float> el_values;
    const float el_min = std::min(config.start_el_deg, config.end_el_deg);
    const float el_max = std::max(config.start_el_deg, config.end_el_deg);
    for (float el = el_min; el <= el_max + 0.5f * el_step; el += el_step) {
      el_values.push_back(el);
    }
    if (el_values.empty()) {
      el_values.push_back(config.start_el_deg);
    }

    const bool start_from_right = (config.start_pos == 1 || config.start_pos == 2);
    const bool start_from_bottom = (config.start_pos == 2 || config.start_pos == 3);
    if (start_from_right) {
      std::reverse(az_values.begin(), az_values.end());
    }
    if (!start_from_bottom) {
      std::reverse(el_values.begin(), el_values.end());
    }

    std::vector<BeamPointingDeg> pattern;
    pattern.reserve(az_values.size() * el_values.size());
    if (config.sequence == 0) {
      for (std::size_t el_index = 0; el_index < el_values.size(); ++el_index) {
        for (std::size_t az_index = 0; az_index < az_values.size(); ++az_index) {
          BeamPointingDeg beam;
          beam.az_deg = az_values[az_index];
          beam.el_deg = el_values[el_index];
          pattern.push_back(beam);
        }
      }
      return pattern;
    }

    for (std::size_t az_index = 0; az_index < az_values.size(); ++az_index) {
      for (std::size_t el_index = 0; el_index < el_values.size(); ++el_index) {
        BeamPointingDeg beam;
        beam.az_deg = az_values[az_index];
        beam.el_deg = el_values[el_index];
        pattern.push_back(beam);
      }
    }
    return pattern;
  }

 private:
  /**
   * @brief 规范化方位角到 [-180, 180] 区间。
   * @param[in] az_deg 输入方位角（单位：deg）。
   * @return 规范化后的方位角（单位：deg）。
   */
  static float NormalizeAzimuthDeg(float az_deg) {
    float normalized = az_deg;
    while (normalized > 180.0f) {
      normalized -= 360.0f;
    }
    while (normalized <= -180.0f) {
      normalized += 360.0f;
    }
    return normalized;
  }
};

}  // namespace intercept
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_SCAN_PATTERN_GENERATOR_H_
