/**
 * @file DetectionRecord.h
 * @brief 定义多源融合的泛型探测记录。
 */

#ifndef ONEQ_FUSION_DETECTION_RECORD_H_
#define ONEQ_FUSION_DETECTION_RECORD_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace fusion {

/**
 * @brief 泛型探测记录（算法不感知 ESR/EOS/AR 具体类型，由业务层适配）。
 * @note 位置/方位/特征三通道可独立存在：带位置记录走空间门限，
 *       仅方位记录走方位相干门限，二者皆无时仅走身份键与特征门限。
 */
struct ONEQ_API DetectionRecord {
  std::uint64_t key{0U};                 /**< 库内身份键（调用方提供并保证跨源一致；0 = 无身份） */
  std::uint32_t source_id{0U};           /**< 源通道标识（由调用方映射） */
  bool has_position{false};              /**< 是否携带位置量测 */
  oneq::coordinate::LlaPositionDegM position{}; /**< 位置量测（度制 LLA） */
  bool has_bearing{false};               /**< 是否携带方位量测 */
  double bearing_az_deg{0.0};            /**< 方位角（单位：deg） */
  double bearing_el_deg{0.0};            /**< 俯仰角（单位：deg） */
  std::vector<double> feature{};         /**< 特征向量（可选，空 = 无特征） */
  double verdict{0.0};                   /**< 判决值（0/1） */
  double quality{0.0};                   /**< 探测质量归一化值（[0,1]） */
};

}  // namespace fusion

#endif  // ONEQ_FUSION_DETECTION_RECORD_H_
