/**
 * @file EsrBoresightChain.h
 * @brief ESR 天线指向合成链薄适配：平台姿态（Body->ENU）+ 天线安装偏置（天线坐标系）。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_ESR_BORESIGHT_CHAIN_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_ESR_BORESIGHT_CHAIN_H_

#include "1q/coordinate/types.h"
#include "common/geometry/BoresightChain.h"

namespace electronic_surveillance_radar {
namespace pipeline {

/**
 * @brief ESR 天线指向合成链（委托公共域 BoresightChain，参考系 = ENU）。
 * @details 合成关系：v_enu = R_enu_body(姿态) · R_body_antenna(安装偏置) · v_antenna。
 *          正安装偏置使光轴偏向机体系正方位/正仰角（与历史"波束角 + 安装偏置"的角度
 *          加法语义同向，也与 AR 前向链 platform_attitude ∘ mount ∘ scan 同语义）；
 *          零姿态 + 单轴安装偏置下与历史加法严格一致，非零姿态下按旋转矩阵严格复合
 *          （2026-08-21 由角度加法近似升级，与 SBIRS/AR 链路收敛）。
 * @note 公共链的 mount 参数语义是 Body->Sensor 坐标旋转（取 R_mount⁻¹ 入链，方向与本
 *       模块"光轴安装偏置"相反）：适配层把安装偏置取反后入链，模块侧不见符号差异。
 * @note ENU->ECEF 方向换算依赖平台位置（geodetic 步骤），不属于安装链，留在调用方
 *       （EsrRfV2FrontEnd 的 TryEnuToEcefDirection 步骤）。ESR 无稳定方式配置
 *       （波束随平台姿态即时跟随，隐含随体稳定），亦无安装失准模型；如后续引入，
 *       按公共链对应原语在适配层扩展。
 */
class EsrBoresightChain {
 public:
  /**
   * @brief 由平台姿态与天线安装偏置构造合成链。
   * @param[in] attitude_enu_body_deg 平台姿态欧拉角（Z-Y-X，Body->ENU，单位：deg）
   * @param[in] antenna_mount_az_deg 天线中心方位相对角（光轴体方位偏置，单位：deg）
   * @param[in] antenna_mount_el_deg 天线中心俯仰相对角（光轴体俯仰偏置，单位：deg）
   */
  EsrBoresightChain(const oneq::coordinate::EulerAnglesDeg& attitude_enu_body_deg,
                    double antenna_mount_az_deg, double antenna_mount_el_deg);

  /**
   * @brief 天线系指向旋转为 ENU 单位光轴向量。
   * @param[in] antenna_az_deg 天线系方位角（单位：deg）
   * @param[in] antenna_el_deg 天线系俯仰角（单位：deg）
   * @return ENU 单位向量（双精度，与历史前端正向路径精度一致）
   */
  oneq::coordinate::Vector3d EnuLosOfAntennaPointing(double antenna_az_deg,
                                                    double antenna_el_deg) const;

 private:
  oneq::common::geometry::BoresightChain chain_;
};

}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_ESR_BORESIGHT_CHAIN_H_
