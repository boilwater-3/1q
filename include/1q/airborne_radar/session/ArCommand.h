/**
 * @file ArCommand.h
 * @brief 机载雷达战术指令类型集合。
 *
 * 战术控制指令（来源模块、指令类型）的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_COMMAND_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_COMMAND_H_

#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief ArCommandSource 表示指令来源模块。
 * 决策层处理结束后根据来源模块进行指令分类分别下发到不同的硬件接口或处理单元。
 */
enum class ONEQ_API ArCommandSource {
  UNKNOWN,    /**< 未知来源或未设置来源 */
  CLASSIFIER, /**< 来源于 Classifier 模块 */
  LPI,        /**< 来源于 LPI 模块 */
  ECCM        /**< 来源于 ECCM 模块 */
};

/**
 * @brief ArCommandType 表示战术指令类型。
 */
enum class ONEQ_API ArCommandType {
  NONE,                        /**< 空指令，占位或无操作 */
  SET_LPI_POWER,               /**< 设置 LPI 发射功率控制 */
  SET_LPI_BEAMFORMING,         /**< 设置 LPI 波束形成参数（当前阶段预留占位，尚未启用） */
  SET_LPI_DWELL,               /**< 设置 LPI 驻留控制参数（当前阶段预留占位，尚未启用） */
  ENABLE_SIDELOBE_CANCELLER,   /**< 开启 ECCM 旁瓣对消 */
  ENABLE_ADAPTIVE_BEAMFORMING, /**< 开启 ECCM 自适应波束形成 */
  SET_AGILITY_FREQ,            /**< 设置 ECCM 频率捷变 */
  SET_ECCM_REJITTER,           /**< 设置 ECCM 重频抖动 */
  SET_ECCM_BURNTHROUGH_GAIN    /**< 设置 ECCM 烧穿增益 */
};

/**
 * @brief ArCommand 表示可携带参数的战术控制指令。
 */
struct ONEQ_API ArCommand {
  ArCommandType type{ArCommandType::NONE};          /**< 指令类型 */
  ArCommandSource source{ArCommandSource::UNKNOWN}; /**< 指令来源模块 */

  ArCommand() = default; /**< 默认构造函数，初始化为无操作指令 */

  /**
   * @brief 构造指令。
   * @param[in] cmd_type 指令类型。
   * @param[in] cmd_source 指令来源模块。
   */
  ArCommand(ArCommandType cmd_type, ArCommandSource cmd_source)
      : type(cmd_type), source(cmd_source) {}
};


}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_COMMAND_H_
