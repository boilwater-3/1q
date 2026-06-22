# SAR 缺失脉冲拒绝参考矩阵契约

## 1. 目标

使用固定参考点场景验证缺失脉冲间隙门禁在二维重采样和 RDA 聚焦前正确停止。首批只
验证允许/拒绝和停止语义，不修复缺失脉冲。

## 2. 参考输入

- 使用固定 `ReferencePointScene`、单点目标和确定性 raw-history。
- expected interval 固定为 `1 / scene.prf_hz`。
- 所有 case 从均匀脉冲序列删除指定行，同时删除对应显式时刻和 raw-history 行。
- 不重新编号、排序或插入样本。

## 3. 首批矩阵

1. `baseline`：不删除脉冲，门禁允许，重采样与 RDA 完成。
2. `small_jitter`：无删除且最大间隙比 `<1.5`，门禁允许，重采样与 RDA 完成。
3. `boundary_1_5x`：人工时间轴包含恰好 `1.5x` 间隙，门禁拒绝。
4. `single_missing`：删除一个内部脉冲，形成约 `2x` 间隙，估计缺失 1，门禁拒绝。
5. `two_adjacent_missing`：删除两个相邻内部脉冲，形成约 `3x` 间隙，估计缺失 2，门禁拒绝。
6. `two_separate_missing`：删除两个不相邻内部脉冲，记录两个拒绝间隙并累计缺失 2。

## 4. 停止语义

- 允许 case：门禁重采样返回成功，输出矩阵非空，随后 RDA 成功。
- 拒绝 case：门禁重采样返回失败，输出矩阵为空。
- 拒绝 case 不得调用 RDA；测试必须以显式 `rda_attempted=false` 证明停止。
- 拒绝不是质量下降路径，不记录或比较拒绝 case 图像。

## 5. 首批验收

1. baseline 与 small jitter 完成重采样和 RDA。
2. boundary、单缺失和多缺失全部拒绝。
3. 单/多缺失的 rejected gap count、suspected missing count 和首个索引正确。
4. 拒绝 case 输出为空且 RDA 未尝试。
5. 输入不变、结果确定，默认与 Eigen 3.3.9 C++11 门通过。

## 6. 后置内容

- 缺失修复、插值补点、稀疏恢复与 NUFFT。
- 生产 RDA、Session、public API、schema、trace、replay 和 Auto。

## 7. 下一实现边界

下一阶段只新增参考矩阵测试与验收报告，不修改生产算法路径。
