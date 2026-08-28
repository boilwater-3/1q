/**
 * @file detection_delivery.h
 * @brief 示例层探测记录投递方式（黑板 / 消息双模式）。
 */

#ifndef EXAMPLES_CORE_DETECTION_DELIVERY_H_
#define EXAMPLES_CORE_DETECTION_DELIVERY_H_

namespace component_attachment {

/** @brief 探测记录投递方式（机载/地基传感器 → 融合消费方）。 */
enum class DetectionDeliveryMode {
  kSharedBlackboard = 0, /**< 写 AppSceneState.detection_pool */
  kMessage = 1           /**< 发 on_detection_batch_submitted */
};

}  // namespace component_attachment

#endif  // EXAMPLES_CORE_DETECTION_DELIVERY_H_
