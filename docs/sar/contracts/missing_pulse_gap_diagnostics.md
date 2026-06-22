# SAR 缺失脉冲与最大慢时间间隙诊断契约

## 1. 目标

在慢时间重采样前诊断大间隙和疑似缺失脉冲，明确允许/拒绝语义。首批只诊断和拒绝，
不修复缺失脉冲，不接入生产 RDA 或 Session。

## 2. 输入边界

- 至少 2 个有限、严格递增的显式脉冲时刻。
- 显式正有限 `expected_interval_s`，来自已审批的名义 PRF/调度。
- 首批固定最大允许间隙比 `1.5`。
- 不允许仅用首尾跨度反推 expected interval；缺失脉冲会污染该估计。

## 3. 间隙与缺失估计

对相邻时刻：

$$
gap_i=t[i+1]-t[i]
$$

$$
gapRatio_i=gap_i/expectedInterval
$$

当 `gapRatio_i >= 1.5` 时，该间隙为拒绝间隙，并估计：

$$
suspectedMissing_i=\max(1,\ round(gapRatio_i)-1)
$$

总疑似缺失数为所有拒绝间隙估计之和。该值仅为诊断，不用于自动插入样本。

## 4. 结构化诊断

至少记录：

- sample count
- expected interval / expected PRF
- minimum / maximum actual gap
- maximum gap ratio
- rejected gap count
- suspected missing pulse count
- first rejected gap index
- resampling allowed status

没有拒绝间隙时 `first_rejected_gap_index` 使用 `size_t` 最大值。

## 5. 允许与拒绝语义

- 所有间隙比 `< 1.5`：允许进入现有慢时间重采样基础。
- 任一间隙比 `>= 1.5`：拒绝重采样。
- 重复、逆序、非有限时刻或无效 expected interval：结构无效并拒绝。
- 拒绝后不得排序、补点、跨间隙插值或静默继续。

## 6. 首批验收矩阵

1. 均匀时间轴允许，最大间隙比为 1，缺失数为 0。
2. 小抖动但最大间隙比小于 1.5 时允许。
3. 单个约 `2x` 间隙估计 1 个缺失脉冲并拒绝。
4. 单个约 `3x` 间隙估计 2 个缺失脉冲并拒绝。
5. 多个拒绝间隙累计缺失数并记录首个索引。
6. 恰好 `1.5x` 边界拒绝。
7. 非法输入拒绝，重复计算确定，双环境 C++11 门通过。

## 7. 后置内容

- 缺失脉冲定位真值、样本插入、稀疏恢复或 NUFFT。
- 随机调度、脉冲 ID 与时刻联合诊断。
- RDA/CSA/Omega-K、Session、public API、schema、trace 和 replay 接入。

## 8. 下一实现边界

下一阶段只实现独立间隙诊断函数，并在独立包装入口中门禁现有向量/二维重采样；不修改
现有无门禁基础函数和生产聚焦路径。
