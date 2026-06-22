# SAR 内部确定性聚焦算法选择器工程契约

## 1. 目标

建立一个内部、无状态、确定性的聚焦算法建议器，在已审批的 RDA、GBP、BP 之间根据
显式输入返回建议与原因。选择器不运行算法、不修改配置、不触发回退，也不开放 public
`Auto`。

## 2. 输入

选择请求至少包含：

- `trajectory_fidelity`：`L1`、`L2` 或 `L3`
- `purpose`：常规成像、独立参考、L3 非直线成像
- `range_sample_count`
- `azimuth_pulse_count`
- `rda_available`
- `gbp_available`
- `bp_available`
- `l2_compensation_enabled`
- `l3_waypoints_available`

所有尺寸必须大于零。调用目的和轨迹等级必须一致，不允许选择器猜测调用方意图。

## 3. 输出

输出至少包含：

- `valid`
- `recommended_algorithm`：`RDA`、`GBP`、`BP` 或 `None`
- `reason`
- `rejection`
- 请求尺寸和轨迹等级回显

输出只是一份建议，调用方必须显式决定是否执行。

## 4. 首批规则

规则按下列顺序执行：

1. 输入无效或目的/轨迹矛盾：拒绝。
2. 常规成像：
   - `L1`：建议 RDA。
   - `L2` 且显式启用一阶补偿：建议 RDA。
   - `L2` 未启用补偿：拒绝，不自动切换 BP。
   - `L3`：拒绝，调用方必须显式选择 L3 非直线成像目的。
3. L3 非直线成像：
   - 必须为 `L3` 且有航路点。
   - 建议 BP。
4. 独立参考：
   - 只允许小场景，建议 GBP。

## 5. 尺寸门

- RDA：`range_sample_count <= 1024` 且 `azimuth_pulse_count <= 1024`。
- GBP：两维均 `<= 128`。
- BP：两维均 `<= 128`。

超限必须拒绝，不允许降级到另一算法。

## 6. 可用性门

建议算法必须显式可用。若对应算法不可用则拒绝，不允许按备用顺序静默选择其他算法。

## 7. 结构化原因

首批建议原因：

- L1 常规 RDA
- L2 补偿 RDA
- 小场景独立 GBP 参考
- L3 航路点 BP

首批拒绝原因：

- 无效尺寸
- 目的与轨迹矛盾
- L2 缺少补偿
- L3 缺少航路点
- 算法不可用
- 算法尺寸超限
- 未支持的请求

## 8. 首批验收矩阵

1. L1 常规、L2 补偿常规、L3 BP、独立 GBP 均返回预期建议。
2. L2 无补偿、L3 常规、L3 无航路点被拒绝。
3. 每条算法不可用时被拒绝，不静默切换。
4. 每条算法尺寸边界内通过，超一档拒绝。
5. 相同请求重复执行返回逐字段一致结果。
6. 选择器不会调用任何聚焦算法或修改输入。

## 9. 实现边界

- 首批实现放入 `src/sar/imaging` 内部模块。
- 不修改 public `FocusingAlgorithm`、policy、Session、schema、trace 或 replay。
- 不使用质量阈值、SNR/SCR、辐射误差或运行时耗时做选择。
- 不支持 CSA、Omega-K、自聚焦、时变 PRF 或 GPU。
- 不启用 public Auto。

## 10. 后续审批

内部选择器完成后，再决定是否建立受控 Session 集成契约。public Auto 必须继续经过
独立审批。
