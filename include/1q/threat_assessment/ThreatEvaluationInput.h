/**
 * @file ThreatEvaluationInput.h
 * @brief 定义威胁评估的泛型输入帧。
 */

#ifndef ONEQ_THREAT_ASSESSMENT_THREAT_EVALUATION_INPUT_H_
#define ONEQ_THREAT_ASSESSMENT_THREAT_EVALUATION_INPUT_H_

#include <cstdint>

#include "1q/api.hpp"

namespace threat_assessment {

/**
 * @brief 单目标的威胁评估输入（算法不感知传感器/坐标系，由调用方组装）。
 * @details 属性侧（运动学/RCS/识别）通常来自机载雷达航迹快照，证据侧
 *          （融合置信度）来自多源融合输出；二者由调用方按目标键对齐后组装。
 *          距离由调用方计算（目标到受保护资产/平台的斜距），算法不假设坐标帧。
 */
struct ONEQ_API ThreatEvaluationInput {
  std::uint64_t key{0U};              /**< 目标库内键（透传，用于输出对齐） */
  float speed{0.0f};                  /**< 速度模长（单位：m/s） */
  float range_m{0.0f};                /**< 目标到受保护资产/平台的斜距（单位：m） */
  float acceleration{0.0f};           /**< 加速度模长（单位：m/s^2） */
  float rcs{0.0f};                    /**< 目标估计雷达散射截面积（单位：m^2） */
  float target_probability{0.0f};     /**< 目标类型/识别概率（单位：1，[0,1]） */
  float fusion_confidence{0.0f};      /**< 融合置信度（融合模块输出，可 >1，评估器内部钳制） */
};

}  // namespace threat_assessment

#endif  // ONEQ_THREAT_ASSESSMENT_THREAT_EVALUATION_INPUT_H_
