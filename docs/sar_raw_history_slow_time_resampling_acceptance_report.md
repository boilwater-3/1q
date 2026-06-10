# SAR 二维 raw-history 慢时间重采样验收报告

## 1. 验收范围

本阶段只验收内部二维 raw-history 逐距离列慢时间线性重采样，不批准 RDA、Session 或
public 接入。

## 2. 实现结果

- 在 `SarSlowTimeResampling` 增加二维 `ComplexMatrix` 重采样入口。
- 保持 row=方位、col=距离和输入输出尺寸一致。
- 所有列复用相同显式时刻与名义轴诊断。
- 每列独立沿方位方向重采样，首尾行保持。
- 非法矩阵尺寸和时刻数量被拒绝。

## 3. 测试证据

- 默认与 Eigen 3.3.9 `SarRawHistorySlowTimeResamplingTest.*`：各 `3/3` passed。
- 默认完整 CTest：`25/25` passed。
- Eigen 3.3.9 `sar_cxx11_compat`：`1/1` passed。
- `git diff --check`：passed。

## 4. 已解决问题

初始实现误用不存在的 `ComplexMatrix(rows, cols)` 构造函数导致编译失败；已按现有类型
模式显式设置 `rows/cols/values`，随后真实执行新增测试并通过。

## 5. 审批结论

二维 raw-history 慢时间重采样完成当前平台审批。带限回波质量阈值、缺失脉冲策略和
RDA/Session 接入继续后置。
