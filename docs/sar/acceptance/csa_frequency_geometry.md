# SAR CSA 频率几何基础验收报告

## 1. 验收范围

本阶段只验收 CSA 未 shift 频率轴、`D(f_a)`、`alpha(f_a)` 与有效域诊断，
不批准完整 CSA 聚焦、相位函数、Session 接入或 public 算法入口。

## 2. 实现结果

- 新增内部 `SarCsaGeometry` 无状态诊断模块。
- 输入显式包含二维尺寸、采样率、PRF、载频、平台速度和参考斜距。
- 输出包含双频率轴、波长、几何因子、chirp-scaling 系数和有效域裕量。
- 越界 Doppler bin 被计数并写入有限占位值，整体诊断标记为无效。
- 结构无效输入被拒绝并清空输出。

## 3. 交叉核对

`SarCsaGeometry` 的方位频率轴与现有 `SarRda.cpp::DopplerFrequency` 使用相同约定：

1. `index <= count / 2` 使用正频率 `index * PRF / count`。
2. 其余 bin 减去 PRF。
3. 偶数长度 Nyquist bin 保持在正频率位置。

该核对只批准频率轴一致性，不代表完整 CSA 与 RDA 成像结果等价。

## 4. 测试证据

- 默认 Windows Debug 构建通过。
- 默认 `SarCsaGeometryTest.*`：`4/4` passed。
- 默认完整 CTest：`25/25` passed。
- Eigen 3.3.9 `SarCsaGeometryTest.*`：`4/4` passed。
- Eigen 3.3.9 `sar_cxx11_compat`：`1/1` passed。
- `git diff --check`：passed。

测试覆盖奇偶长度未 shift 轴、零 Doppler、正负对称性、接近边界趋势、
无效域有限输出、结构无效输入和确定性。

## 5. 审批结论

CSA 频率几何基础完成当前平台审批。完整 chirp-scaling、SRC/RCMC、方位压缩和
残余相位函数仍缺少独立中间域真值，继续后置。
