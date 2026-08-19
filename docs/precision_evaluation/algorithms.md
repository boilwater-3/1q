---
Status: active
Last-reviewed: 2026-08-19
---

# 精度评估算法登记

本文是精度评估算法清单与口径的权威；逐步逻辑读代码（`src/precision_evaluation/`）。
模块边界见 [boundaries.md](boundaries.md)，导航见 [design.md](design.md)。

## 算法登记表

| 算法 | 意图（一句话） | 实现状态 | 证据 |
|---|---|---|---|
| 双视线交会（双星三角定位） | 两条异面视线最近点中点为定位解，最近距离为几何残差 | 生产可用 | [evidence: tests/unit/precision_evaluation/precision_evaluation_math_test] |
| 误差序列汇总 | mean/RMSE/P95（最近秩法）/max 四统计 | 生产可用 | [evidence: tests/unit/precision_evaluation/precision_evaluation_math_test] |
| AHP 判断矩阵求解 | 幂迭代主特征向量权重 + λmax/CI/CR 一致性（Saaty，RI 表 n≤5 固化） | 生产可用 | [evidence: tests/unit/precision_evaluation/precision_evaluation_math_test] |
| 综合精度评分 | score=ref/(ref+rmse)，composite=Σw·score，贡献分解 | 生产可用 | [evidence: tests/unit/precision_evaluation/precision_evaluation_math_test] |
| 双星编排评估会话 | 双 SBIRS 会话 + fusion + 推演的逐周期误差提取、五指标汇总与 AHP 评分 | 生产可用（2026-08-19） | [evidence: tests/unit/precision_evaluation/precision_evaluation_session_test] |

## 公式与口径

- **双 LOS 交会**：`denom = |Da×Db|² = a·c − b²`（a=Da·Da、b=Da·Db、c=Db·Db）；
  `t=(b·e−c·d)/denom`、`s=(a·e−b·d)/denom`（d=Da·w0、e=Db·w0、w0=Oa−Ob）；
  定位解 = 两最近点中点，残差 = 最近点距离。denom 近零（平行）判不可解。
- **P95**：升序第 `ceil(0.95·n)` 个样本（1 基最近秩）；短序列退化为最大值。
- **AHP**：正互反矩阵（对角 1、a_ij·a_ji=1、元素∈[1/9,9]，容差 1e-6）→ 幂迭代
  （初值全 1，归一化迭代至 Δ≤1e-12 或 1000 次）得 Perron 权重 → Rayleigh 商 λmax →
  `CI=(λmax−5)/4`、`CR=CI/1.12`；`is_consistent = CR ≤ 0.1`。默认矩阵全 1 = 等权
  完全一致。**一致性不满足仍返回权重但置 false**，调用方须提示重标定。
- **归一化评分**：`score = ref/(ref+rmse)`（rmse=ref 得 0.5）；参考误差默认值为演示
  口径（角度 0.05°、双星 10 km、速度 100 m/s、落点 10 km、发射点 20 km），正式验收
  标定前按装备指标替换。

## 编排会话与日志事件

- **编排流程**（`PrecisionEvaluationSession::Step`，每周期）：同目标场景、同 GMST 驱动两个
  独立 SBIRS 会话 → ①角度误差（各星输出角 vs 真值角，方位最短角差）；②双星同目标双检
  出时双 LOS 交会 → 位置误差 + 几何残差；③双星检测经适配器（归属层 target_id 恢复身份键
  + ECI 角 → 卫星局部 ENU 方位/原点）进 `FusionEngine::Update`，逐航迹运动学估计 → 速度
  误差（附位置误差）；④按 `inference_interval_cycles` 以估计状态与真值状态分别喂
  `TargetInferenceEngine::Infer`（真值关键点每目标缓存一次）→ 落点/发射点误差。
  `Summarize()` 聚合五指标并求 AHP 权重与综合分（矩阵非法 → `ahp_valid=false`、综合分 0，
  不静默退化；**零证据=零分**：count=0 的指标 rmse 按 +∞ 进评分，得 0 分拖低综合，
  不因空序列 rmse=0 而得满分）。
- **日志事件**（`[PrecisionEval]` 前缀，开关 `ONEQ_ENABLE_PRECISION_EVALUATION_LOG` 默认
  OFF）：`angular_error` / `dual_sat_fix` / `velocity_error` / `keypoint_error`（逐周期
  样本）、`metric_summary` / `ahp_score`（Summarize 时）。仅人读验收材料，非机器契约。
- **配置保证**（会话构造时统一生效，调用方无须自查）：`fusion.enable_track_filtering`
  强制 true；双星源通道 id 互异（B 星撞 A 星 id 时 +100）；`inference_interval_cycles`
  钳制 ≥1。

## 反直觉点

1. AHP 判断矩阵默认全 1 时 CR=0 且等权——"没配置"不等于"没评估"，但等权本身是
   一个显式假设，正式验收须替换为专家标定矩阵。
2. 双 LOS 残差不是误差：残差是两视线几何不交的程度（交会虚实），位置误差才是对真值
   的偏差；两者都进样本供分辨"几何构型差"与"量测噪声大"。
