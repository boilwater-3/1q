# SAR Omega-K Stolt 几何基础验收报告

## 1. 验收范围

本阶段只验收 Omega-K 双程波数、传播色散有效域、Stolt 源距离频率查询和诊断，
不批准复数谱插值、参考相位、完整聚焦或 Session/public 接入。

## 2. 实现结果

- 新增内部 `SarOmegaKGeometry` 无状态诊断模块。
- 生成与 RDA/CSA 一致的未 shift 双频率轴。
- 计算 `K_r`、`K_x`、有效 `K_z` 和均匀目标 `K_z` 对应的源距离频率查询。
- 分别计数色散无效点和 Stolt 越支持区查询。
- 无效点使用有限占位值，不产生 NaN/Inf。
- 零 Doppler 路径直接复用原距离频率，保证 Stolt shift 精确为零。

## 3. 测试证据

- 默认 Windows Debug 构建通过。
- 默认 `SarOmegaKGeometryTest.*`：`5/5` passed。
- 默认完整 CTest：`25/25` passed。
- Eigen 3.3.9 `SarOmegaKGeometryTest.*`：`5/5` passed。
- Eigen 3.3.9 `sar_cxx11_compat`：`1/1` passed。
- `git diff --check`：passed。

测试覆盖未 shift 轴、零 Doppler、正负对称、shift 单调趋势、色散无效点、
越支持区查询、有限输出、非法输入和确定性。

## 4. 已解决问题

初始实现的零 Doppler 查询经过波数到频率反算后产生约 `1.19e-7 Hz` 舍入残差。
实现改为零方位波数时直接复用原距离频率，使零 Stolt shift 成为精确不变量。

## 5. 审批结论

Omega-K Stolt 几何基础完成当前平台审批。复数谱插值、支持区处理策略、参考相位和
完整聚焦仍缺少独立中间域真值，继续后置。
