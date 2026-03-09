// Copyright 2026. All Rights Reserved.
//
// Description: 定义行为决策层可以下发的指令。

#ifndef AIRBORNE_RADAR_COMMON_RADAR_COMMAND_H_
#define AIRBORNE_RADAR_COMMON_RADAR_COMMAND_H_

# include <boost/any.hpp>

namespace airborne_radar {
namespace common {

/// @brief RadarCommandSource 表示指令来源模块。
/// 决策层处理结束后根据来源模块进行指令分类分别下发到不同的硬件接口或处理单元。
enum class RadarCommandSource {
  /// @brief 未知来源或未设置来源。
  UNKNOWN,

  /// @brief 来源于 Classifier 模块。
  CLASSIFIER,

  /// @brief 来源于 LPI 模块。
  LPI,

  /// @brief 来源于 ECCM 模块。
  ECCM
};

/// @brief RadarCommandType 表示战术指令类型。
enum class RadarCommandType {
  /// @brief 空指令，占位或无操作。
  NONE,

  /// @brief 设置 LPI 发射功率控制。
  SET_LPI_POWER,

  /// @brief 设置 LPI 波束形成参数（当前阶段预留占位，尚未启用）。
  SET_LPI_BEAMFORMING,

  /// @brief 设置 LPI 驻留控制参数（当前阶段预留占位，尚未启用）。
  SET_LPI_DWELL,

  /// @brief 开启 ECCM 旁瓣对消。
  ENABLE_SIDELOBE_CANCELLER,

  /// @brief 开启 ECCM 自适应波束形成。
  ENABLE_ADAPTIVE_BEAMFORMING,

  /// @brief 设置 ECCM 频率捷变。
  SET_AGILITY_FREQ,

  /// @brief 设置 ECCM 重频抖动。
  SET_ECCM_REJITTER,

  /// @brief 设置 ECCM 烧穿增益。
  SET_ECCM_BURNTHROUGH_GAIN
};

/// @brief RadarCommand 表示可携带参数的战术控制指令。
struct RadarCommand {
  /// @brief 指令类型。
  RadarCommandType type{RadarCommandType::NONE};

  /// @brief 指令来源模块。
  RadarCommandSource source{RadarCommandSource::UNKNOWN};

  /// @brief 指令伴随信息。
  boost::any info;

  /// @brief 默认构造函数，初始化为无操作指令。
  RadarCommand() = default;

  /// @brief 构造函数，初始化指令类型和来源。
  RadarCommand(RadarCommandType cmd_type, RadarCommandSource cmd_source,
               const boost::any &cmd_info = boost::any())
      : type(cmd_type), source(cmd_source), info(cmd_info) {}

};
} // namespace common
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_COMMON_RADAR_COMMAND_H_
