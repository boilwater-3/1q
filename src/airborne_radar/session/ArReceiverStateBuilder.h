/**
 * @file ArReceiverStateBuilder.h
 * @brief 无状态构造器——从 prepare 输入、发射事实与工程配置组装接收工作状态。
 *
 * 与 ArEmissionFactory 同型：纯函数，不持有状态。输出 ArReceiverOperatingState
 * 供 PreparedCycleLedger::CommitPrepared 冻结，CompleteRfCycle 全程只读消费。
 */

#ifndef AIRBORNE_RADAR_SESSION_AR_RECEIVER_STATE_BUILDER_H_
#define AIRBORNE_RADAR_SESSION_AR_RECEIVER_STATE_BUILDER_H_

#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "1q/airborne_radar/session/ArControlProfile.h"
#include "airborne_radar/session/ArRfCycleState.h"
#include "1q/electromagnetics/RfScene.h"

namespace airborne_radar {
namespace session {

/**
 * @brief 无状态接收状态构造器：组装 rf_receiver（identity/位置/天线/极化/窗口/频率/
 * 带宽/损耗/远场/co-site）与 operating_state（波束指向/匹配滤波/噪声系数/线性输入功率）。
 *
 * 天线从发射拷贝，再按控制档施加旁瓣对消（-12 dB）与自适应波束形成（波束压缩 0.75、
 * 旁瓣 -6 dB）效果。所有计算均为 prepare 期冻结的确定函数，不含随机状态。
 */
class ArReceiverStateBuilder {
 public:
  /**
   * @brief 组装接收工作状态。
   *
   * @param input prepare 输入（platform 身份/位置/速度/波束指向/窗口）。
   * @param emission 已构造的发射事实（天线方向/极化从其拷贝）。
   * @param detection 工程配置（接收机/发射机带宽）。
   * @param control_profile ECCM 控制档（旁瓣对消/自适应波束形成开关）。
   * @param carrier_hz 已解析载频。
   * @return 冻结的接收工作状态。
   */
  static ArReceiverOperatingState Build(const ArPrepareCycleInput& input,
                                        const oneq::electromagnetics::RfSceneEmission& emission,
                                        const config::engineering::DetectionConfig& detection,
                                        const ArControlProfile& control_profile,
                                        double carrier_hz);
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_AR_RECEIVER_STATE_BUILDER_H_
