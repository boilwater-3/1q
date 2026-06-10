# SAR 内部确定性聚焦算法选择器审批报告

## 1. 审批结论

阶段 68 内部确定性聚焦算法选择器已完成当前平台审批。

- L1 常规成像建议 RDA。
- L2 显式一阶补偿常规成像建议 RDA。
- 小场景独立参考建议 GBP。
- L3 显式航路点非直线成像建议 BP。
- 前置条件缺失、算法不可用和尺寸超限均明确拒绝，不静默切换。
- 相同请求重复执行逐字段一致，且选择器不修改请求或运行聚焦算法。

## 2. 实现范围

- 新增内部模块 `src/sar/imaging/SarFocusingSelector.{h,cpp}`。
- 新增轨迹等级、调用目的、算法可用性、尺寸门和结构化原因。
- 未修改 public API、Session、schema、trace 或 replay。

## 3. 验证结果

- 默认环境选择器测试：`4/4 passed`。
- Conan Eigen 3.3.9 选择器测试：`4/4 passed`。
- 默认 Windows Debug 完整 CTest：`25/25 passed`。
- Conan Eigen 3.3.9 `sar_cxx11_compat`：`1/1 passed`。

## 4. 冻结边界

- 当前只批准内部建议器，不批准 public Auto。
- 选择器不执行算法、不修改配置、不回退。
- 不使用质量阈值、SNR/SCR、辐射误差或运行时耗时决策。
- 不支持 CSA、Omega-K、自聚焦、时变 PRF 或 GPU。

## 5. 下一阶段建议

进入内部选择器后续接入决策门，审计是否可以受控接入 Session 内部诊断，或是否应先
扩展选择矩阵与结构化记录。
