# SAR 内部慢时间重采样请求与执行边界契约

## 1. 目标

将已审批的间隙门禁与二维慢时间重采样组合为可审计的内部显式请求执行器。首批只产出
重采样 raw-history 和结构化诊断，不调用 RDA，不接入 Session 或 public 表面。

## 2. 请求输入

内部请求必须显式提供：

- `request_id`：非零内部请求标识。
- `explicit_times_s`：有限、严格递增的脉冲时刻。
- `expected_interval_s`：正有限名义间隔。
- `raw_history`：row=方位、col=距离的完整 `ComplexMatrix`。

执行器不得从首尾跨度猜测 expected interval，不得修改请求输入。

## 3. 执行状态与原因

状态：

- `kSucceeded`
- `kRejected`

原因：

- `kNone`
- `kInvalidRequestId`
- `kInvalidExpectedInterval`
- `kInvalidTimeAxis`
- `kInvalidRawHistory`
- `kMissingPulseGap`
- `kResamplingFailure`

首批不自动修复或回退。

## 4. 输出

执行结果至少包含：

- request id
- status / reason
- gap diagnostics
- resampling diagnostics
- resampled raw-history

成功时输出矩阵完整有效。拒绝时输出矩阵必须为空。所有输出一次性构造，失败不得留下
部分矩阵。

## 5. 执行顺序

1. 验证 request id、expected interval、时间轴和 raw-history 结构。
2. 执行缺失脉冲间隙诊断。
3. 间隙拒绝时返回 `kMissingPulseGap`。
4. 调用二维慢时间重采样基础。
5. 成功后原子发布完整结果。

## 6. 首批验收矩阵

1. baseline 与小抖动请求成功，输出完整。
2. 缺失脉冲请求以 `kMissingPulseGap` 拒绝且输出为空。
3. 非法 request id、expected interval、时间轴和矩阵分别返回对应原因。
4. 输入请求在执行前后保持不变。
5. 重复执行确定，默认与 Eigen 3.3.9 C++11 门通过。

## 7. 后置内容

- RDA/CSA/Omega-K 调用与 Auto。
- Session、public API、schema、trace、replay 和 runtime patch。
- 缺失修复、NUFFT、sinc/高阶慢时间插值。

## 8. 下一实现边界

下一阶段只实现 `src/sar/imaging` 内部无状态执行器和单元测试。
