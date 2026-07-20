# SBIRS TruthAssisted 模式冻结讨论

Status: draft
Review-Date: 2026-07-20
Authority: non-normative Stage A discussion
Scope: TruthAssisted mode semantics only

## 1. 已冻结边界

本草案只保留尚未裁决的 TruthAssisted 双模式问题。环形 WFOV 扫描、runtime patch 状态影响、
组件所有权、滤波后端评估表、物理链 defer 和输出术语已经迁入
`docs/space_based_infrared_sensor/design.md`，不在本草案形成第二权威。

本轮不改变现有 `enable_estimated_tracking=false`、`kTruthAssistedTracking`、public DTO、replay schema
或随机流语义。

## 2. 当前事实

当前 TruthAssisted 路径不创建滤波状态；真值 LOS 同时驱动 NFOV 命令和成功检测角度，但仍受实际
actuator LOS、NFOV 几何门、SNR 门、coasting 和连续失败丢锁约束。

权威设计仍写有“输出测量可叠加显示噪声”，live 实现没有这条独立输出噪声路径。该差异在裁决前不得
被解释成已实现能力。

## 3. 已选择但未实施的方向

审批人选择保留 strict-truth oracle，并另开 Stage A 讨论独立的 sensor-like truth-assisted 候选模式，
而不是静默改变现有模式。

继续讨论时必须冻结：

1. 候选模式是测试用受控替身还是产品可选跟踪模式。
2. 真值只驱动内部命令，还是也参与可见性/状态转移。
3. 输出误差是否消费现有测量随机流，及其 snapshot/replay 归属。
4. public 配置、attribution/debug 与内部状态是否需要区分两个 truth 模式。
5. 默认路径仍是否为 EstimatedTracking。

在上述问题裁决前，不修改 TruthAssisted 生产语义。
