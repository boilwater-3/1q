/**
 * @file RadarCommand.h
 * @brief 定义行为决策层可以下发的指令。
 */

#ifndef AIRBORNE_RADAR_COMMON_RADAR_COMMAND_H_
#define AIRBORNE_RADAR_COMMON_RADAR_COMMAND_H_

namespace airborne_radar {
namespace common {
namespace control {

/**
 * @brief RadarCommandSource 表示指令来源模块。
 * 决策层处理结束后根据来源模块进行指令分类分别下发到不同的硬件接口或处理单元。
 */
enum class RadarCommandSource {
  UNKNOWN,    /**< 未知来源或未设置来源 */
  CLASSIFIER, /**< 来源于 Classifier 模块 */
  LPI,        /**< 来源于 LPI 模块 */
  ECCM        /**< 来源于 ECCM 模块 */
};

/**
 * @brief RadarCommandType 表示战术指令类型。
 */
enum class RadarCommandType {
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
 * @brief RadarCommand 表示可携带参数的战术控制指令。
 */
struct RadarCommand {
  RadarCommandType type{RadarCommandType::NONE};          /**< 指令类型 */
  RadarCommandSource source{RadarCommandSource::UNKNOWN}; /**< 指令来源模块 */

  RadarCommand() = default; /**< 默认构造函数，初始化为无操作指令 */

  /**
   * @brief 构造指令。
   * @param[in] cmd_type 指令类型。
   * @param[in] cmd_source 指令来源模块。
   */
  RadarCommand(RadarCommandType cmd_type, RadarCommandSource cmd_source)
      : type(cmd_type), source(cmd_source) {}
};
}  // namespace control
}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_RADAR_COMMAND_H_
