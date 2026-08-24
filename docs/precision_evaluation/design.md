---
Status: active
Last-reviewed: 2026-08-20
Authority: precision_evaluation 设计权威入口
Answers: 精度评估模块是什么、在五层架构里站在哪、设计文档怎么导航
---

# 精度评估设计

`precision_evaluation` 是评估层模块：拿**场景真值**对照**各层产品**提取定位精度误差
（红外角度误差、双星交会位置误差、速度误差、落点/发射点预测误差），并用层次分析法
（AHP）把五项误差加权成评价卫星红外设备定位精度的指标体系（需求映射 3.2.1.6.3 /
3.2.1.6.3.1 / 3.2.1.6.3.2）。

心智模型：**站在所有产品层之上的验收视角**——传感器/估计/推演层生产产品，本层拿产品
与真值对账。真值只进本层、只用于对照，不回写任何产品层（分层契约去真值化规则的评估
出口）。

完整内容见同目录 [boundaries.md](boundaries.md)（边界与非目标）与
[algorithms.md](algorithms.md)（算法登记、公式与口径）；编排会话与日志事件在
algorithms.md 与代码内注释中描述（暂不设 data-flow.md，同 target_inference 先例，
待跨周期状态复杂化后再引入）。集成参考示例见 `examples/scenes/sbirs_dual_sat_fix/`
（两卫星实体 + 地面站融合组件挂评估会话）。

跨模块公共规则见 `docs/common/contract.md`（分层契约表含评估层行）。
