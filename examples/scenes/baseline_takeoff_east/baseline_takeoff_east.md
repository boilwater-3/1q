# 场景预期表：baseline_takeoff_east（基线）

首次用本模板的样本，同时验证场景文件数据驱动改造无行为漂移（与 README 记载一致）。

### 场景元数据

| 项 | 值 |
| --- | --- |
| 场景文件 | `scenes/baseline_takeoff_east/baseline_takeoff_east.json` |
| 场景意图 | 被测通道：全通道（AR/ESR/EOS/SBIRS/SAR/Fusion/Flight）；被测行为：端到端链路（探测→融合→决策）基线成立；验证深度：L2 + L3 |
| 构建模式 | release；**FD 开**（`ONEQ_ENABLE_FLIGHT_DYNAMIC=ON`） |
| 日志模式 | 视图 delta（默认）+ 事件 key（默认） |
| 输出目录 | `/tmp/1q/scenes/baseline` |
| 确定性 | 固定种子（esr timing_seed=42）→ 实测可复现 |
| 运行日期 | 2026-08-07（改造后基线，新旧代码逐行 diff 一致） |

### 预期事件表

| 通道 | 行为 | 预期周期窗 | 预期量级 | 预期计数 | 实测 | 判定 |
| --- | --- | --- | --- | --- | --- | --- |
| Flight | 起飞爬升到 400 m | cycle 1 起，~cycle 184 达巡航高度 | alt 0→400 m | — | cycle 184 alt=400.4 | 通过 |
| Flight | 航点 0 到达（waypoint_reached） | 演示窗口内（FD 巡航段前） | 距离 < 500 m | 1 | cycle 325（492.6 m） | 通过 |
| AR | 双目标首确认 | cycle 1（目标恒在波束内） | 位置纬度 ≈ 30+range/111 km | 2（1001/1002） | cycle 1 双确认 | 通过 |
| AR | 失跟（target_lost） | 无（目标恒可见） | — | 0 | 0 | 通过 |
| ESR | 假设（emitter_hypothesis） | **KEY 模式不可观测**（DUP 不落盘）；all 模式应可观测 | 频率 9.5/10.0 GHz 分选 | ≥ 2 条假设 | 未观测（模式门控） | 通过（注明） |
| EOS | 首发现/丢失交替 | cycle 101 起（FD 起飞段 cycle 1-100 平台高度不足 + 方位出扫描扇区） | SNR ~25 dB，方位 59-60°（目标近正北） | 首发现 150、丢失 148 | cycle 101 首个首发现（25.1 dB）；150/148 | 通过 |
| SBIRS | 首发现/丢失/跟踪锁定 | cycle 11 起（扫描相位 36 周期一圈门控 + 目标 v_north ±5 穿越质心，见结论注） | SNR 20.0/21.7（linear，≈13 dB） | 24 条（首发现 12、丢失 11、锁定 1） | 24 条 | 通过 |
| SAR | 成像（L1 图像=有） | cycle 19-397 间歇成像（积累 10.24 s 完成后 + squint 随 FD 航迹摆动间歇满足） | SNR 1.3-13.9 dB；目标数 2 | ~198 个周期（README 记载 ~200） | 198 | 通过 |
| SAR | 产品事件 | 首产品 cycle 19（KEY 模式只落关键 1 条） | 阶段=L1 RDA | 1（key） | cycle 19 | 通过 |
| SAR | squint 拒绝（按设计拒绝） | 起飞/转弯段及间歇期 | `squint_angle_exceeds_limit` | 预期大量出现 | 出现（每拒绝周期） | 通过（按设计拒绝） |
| Fusion | 融合目标出现 | cycle 1 起 | fused 2→23（置信度滑窗累积） | 峰值 ≥ 2 | 峰值 23 | 通过 |
| Decision | command_issued | 置信度 ≥ 3.0 首达 | 一次（事件链只下发一次） | 1 | cycle 2 | 通过 |

### 几何先验核对（L3）

| 先验 | 手算值 | 实测来源 | 判定 |
| --- | --- | --- | --- |
| 目标 1001 纬度 ≈ 30.0 + 12/111 km ≈ 30.10811° | 30.10811 | AR 确认事件位置 30.10824 | 通过（差 14 m，ENU→ECEF→LLA 椭球效应量级正确） |
| 目标 1002 纬度 ≈ 30.0 + 14/111 km ≈ 30.12613° | 30.12613 | AR 确认事件位置 30.12628 | 通过 |
| EOS 距离窗 = 高度/sin(俯仰角) = 400/sin(1°~2°) ≈ [11.5, 22.9] km | 12/14 km 在内 | EOS cycle 101 起稳定探测（平台高度爬升足够后） | 通过 |
| 目标恒在平台正北侧方（v_east 47 = 巡航地速）→ AR 方位稳定 | — | AR cycle 1 即确认 | 通过 |

### 冒烟下限（场景 `smoke` 块）

min_key_events=1 / min_sbirs_events=1 / min_sar_products=1 / min_fused_targets=1（默认值，实测远高于下限）

### 结论

**判定：通过**。端到端链路（探测→融合→决策）基线成立，全部预期项满足；与 README
记载一致（EOS ~300 条 cycle 101 起、SAR 成像 ~200 周期、航点 0 窗口内到达），证明
场景文件数据驱动改造无行为漂移（新旧代码逐行 diff 一致）。

**SBIRS 行注（2026-08-08 原型 6 探针修正）**：基线 SBIRS 24 条事件并非"星下点凝视
全覆盖"——实际驱动是：① WFOV 扫描相位门控（360°/10°每周期 = 36 周期一圈，每目标
只在扫描覆盖窗口 2-3 周期被 WFOV 发现）；② targets `v_north_mps=±5` 使两目标 10 m/s
相对接近、cycle ~200 穿越质心 → 星下点附近 az 对 1 km 级偏移极敏感 → az 从 120°/-60°
翻转 → 1002 穿越瞬间 NFOV 几何门失败 → coasting（cycle 201）→ 连续失败 2 周期丢锁
（202）→ 209 重捕获；③ 唯一 NFOV 通道被 1002 占用 → 1001 每 36 周期只走调度跳过
（kSchedulerSkipped）形态：首发现 + 3 周期后无记录（kLost）。详见
`scenario_archetypes.md` 原型 6（原 `sbirs_altitude_snr_1000km` 专项场景已随
2026-08-28 场景集精简移除，结论见原型库）。
