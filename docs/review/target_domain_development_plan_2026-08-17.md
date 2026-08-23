---
Status: final
Date: 2026-08-17
Completed: 2026-08-17（P1-P5 交付完成；P0 残留=需求方指标签认，原 TARGET-OQ-3 已收敛删除，见 §2）
Review-Baseline: `main` @ `f7512cec`（docs: 分层契约冻结 + 存量偏离登记）
Authority: 目标域能力（估计层/推演层）交付计划——交付已完成，本文转为交付记录。
  规定性规则见 docs/common/contract.md §目标处理分层契约；
  需求术语澄清与裁定见 target_domain_requirements_alignment_2026-08-17.md；
  P0-P5 逐阶段终态见 target_domain_p0_p1_decision_2026-08-17.md §6；
  存量偏离登记见 docs/common/open_questions.md（TARGET-OQ-1..4）。
---

# 目标域能力开发计划（SBIRS + RIR 交付范围，已完成）

## 0. 交付范围与结论

> **后续状态（2026-08-17 P0-P5 交付完成，`b9da39d9` 收尾回写）**：估计层
> （fusion 演进）与推演层（新模块 target_inference）全部落地；进度与 commit
> 见 §1。P0 的需求方指标签认仍未闭环（原 TARGET-OQ-3，OQ 条目已收敛删除，
> 裁定内容落 sbirs boundaries 输出规则 4 + fusion algorithms 噪声通道登记 +
> 决策记录 §4.1），残留清单见 §2。
> 各阶段工作项与验收细目已收口进决策记录 §6 终态表，本文不再保留规划正文。

- **交付范围**：以 SBIRS（角度量测源）与 RIR（识别结论源）为传感器输入，建设
  估计层（fusion 演进：关联已有 + 无迹航迹滤波 + 航迹管理）与推演层
  （target_inference：轨迹预测、发射点/落点回推、类型概率融合）。
- **Out of scope（交付时明确不做，边界仍有效）**：AR/ESR/EOS 改动与其适配器
  语义变化；TARGET-OQ-1/2 债务处置（另立分支进行中）；CSO 密集目标分辨；
  传感器模块内部进入估计/推演逻辑（分层契约规则 2/3）；threat_assessment 改动。
- **交付内两个裁定**：TARGET-OQ-3（P2 阻塞）→ 量测噪声通道落地为记录级
  `bearing_noise_sigma_rad` + 配置默认，共模偏差项登记为后续；TARGET-OQ-4
  （P3 阻塞）→ RIR 接入按方案 a（调用方键映射 → type_evidence，零库内改动），
  后经 `rir_dual_product_stage_a_2026-08-18.md` 修订为双产品架构。

## 1. 阶段与进度（完成表）

| 阶段 | 名称 | 状态 | 证据 |
|---|---|---|---|
| P0 | 需求基线与证据矩阵 | 证据已落地；**需求方指标签认未闭环** | 决策记录 §4/§5：OQ-3 语义差实测 + 可达性矩阵实测（地板公里级；σ=5 µrad 触 float 精度边缘）；签认表已填数字 |
| P1 | 无迹滤波原语 | 完成 | 无迹原语三头 + 9 用例；`e6b0aad1`；`unit::common` 全绿 |
| P2 | 估计层 fusion 航迹滤波与管理 | 完成 | 逐航迹无迹滤波 + 航迹管理（默认关零回退；量测原点 ENU 契约 + FusedTarget 运动学/生命周期扩展）；`51c87d70`（边界冻结）+ `0b1a1d6a`；`unit::fusion` 全绿 |
| P3 | 推演层 target_inference | 完成 | 弹道 RK4 前向/回推 + 敏度误差预算 + 类型融合，四处守护注册齐；`9d402196`；`unit::target_inference` 全绿 |
| P4 | 守护、示例与跨层集成 | 完成 | 方向纯净度守护 `check_target_layer_purity.cmake` + component_attachment 推演组件扩链；`3a798c29`；batch 级场景验证以单测 characterization 承载 |
| P5 | 验收与回写 | 完成 | README 模块清单、决策记录 Stage C 终态、全量验证（决策记录 §6） |

## 2. 残留（截至收口时点）

| 项 | 状态 | 载体 |
|---|---|---|
| P0 需求方指标签认 + Estimated 输出语义正式冻结（原 TARGET-OQ-3，条目已收敛删除） | open | 本表 + 决策记录 §4.1（P0 建议裁定：维持 boundaries 输出规则 4 装备语义；估计层默认消费 Sensor-like） |
| TARGET-OQ-1/2 历史越层债务 | 处置进行中（独立分支） | `docs/common/open_questions.md`；依据 `ar_track_attribution_2026-08-21.md` |
| 无迹原语 float 精度（σ≲10 µrad 不足） | P2 冻结项候选 | 决策记录 §6 残留风险 |
| Sensor-like 共模偏差项 R 建模 | 后续 | 同上（待签认口径） |
| batch_validation 场景扩展 | 后续项 | 本表（原 §5 P4 行） |
