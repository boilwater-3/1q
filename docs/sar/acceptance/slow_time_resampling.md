# SAR 时变 PRF 慢时间重采样基础验收报告

## 1. 验收范围

本阶段只验收显式慢时间轴诊断、名义均匀轴和复数向量线性重采样，不批准非均匀
RDA/CSA/Omega-K 成像、缺失脉冲修复或 Session/public 接入。

## 2. 实现结果

- 新增内部 `SarSlowTimeResampling` 无状态模块。
- 使用首尾跨度构造同样本数的名义均匀慢时间轴。
- 输出间隔偏差、时间轴偏差、名义 PRF 和均匀容差诊断。
- 在原时间支持区内执行复数线性重采样。
- 首尾查询严格复用输入首尾样本。
- 重复、逆序、非有限时刻和样本数不匹配被拒绝。

## 3. 测试证据

- 默认与 Eigen 3.3.9 `SarSlowTimeResamplingTest.*`：各 `4/4` passed。
- 默认完整 CTest：`25/25` passed。
- Eigen 3.3.9 `sar_cxx11_compat`：`1/1` passed。
- `git diff --check`：passed。

测试覆盖均匀严格退化、非均匀诊断、首尾保持、复数仿射解析真值、非法输入和确定性。

## 4. 审批结论

时变 PRF 慢时间重采样基础完成当前平台审批。二维 raw-history 重采样、插值误差阈值、
缺失脉冲策略和 FFT 成像接入继续后置。
