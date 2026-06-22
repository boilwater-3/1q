# SAR 内部慢时间重采样请求执行器验收报告

日期：2026-06-11

## 验收范围

阶段 95 实现内部无状态慢时间重采样请求执行器。执行器组合已审批的最大间隙诊断与
二维 raw-history 重采样，并以结构化状态和原因返回结果。

本阶段不接入 RDA、Session、public API、schema 或 replay。

## 实现结果

- 显式接收 request id、慢时间轴、expected interval 和 raw-history。
- 对请求结构、缺失脉冲间隙和重采样失败返回独立拒绝原因。
- 缺失脉冲或结构无效时拒绝请求，并保持输出矩阵为空。
- 成功时一次性返回完整重采样时间轴与 raw-history。
- 执行不修改输入；相同请求重复执行得到确定性结果。

## 验证证据

- 默认环境 `SarSlowTimeResamplingExecutorTest.*`：4/4 passed。
- Eigen 3.3.9 环境 `SarSlowTimeResamplingExecutorTest.*`：4/4 passed。
- 默认环境完整 CTest：25/25 passed。
- Eigen 3.3.9 环境 `sar_cxx11_compat`：1/1 passed。
- Eigen 3.3.9 构建：passed。

## 验收结论

阶段 95 在当前平台通过。执行器仅批准为内部显式入口；生产链接入仍需独立决策门。
