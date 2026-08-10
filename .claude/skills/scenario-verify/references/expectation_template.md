# 预期事件表模板（Expectation Template）

每个场景在**运行前**填好预期列（第 3-5 列），运行后填实测与判定列。
预期表 + 场景 JSON 一起归档，是场景成为回归资产的最小单元。

## 空模板

### 场景元数据

| 项 | 值 |
| --- | --- |
| 场景文件 | `scenes/<name>/<name>.json` |
| 场景意图 | 被测通道：…；被测行为：…；验证深度：L1 冒烟 / L2 预期表 / L3 物理一致性 |
| 构建模式 | release；FD 开 / 关（`ONEQ_ENABLE_FLIGHT_DYNAMIC`） |
| 日志模式 | 视图 delta / summary / nonnominal；事件 key / all / aggregate |
| 输出目录 | `log/<name>` |
| 确定性 | 种子固定（ESR timing_seed、SBIRS/SAR JSON 种子）→ 同场景可复现 |
| 运行日期 | … |

### 预期事件表

| 通道 | 行为 | 预期周期窗 | 预期量级 | 预期计数 | 实测 | 判定 |
| --- | --- | --- | --- | --- | --- | --- |
| Flight | … | | | | | |
| AR | … | | | | | |
| ESR | …（注明：KEY 模式下假设不落盘，需 all/aggregate） | | | | | |
| EOS | … | | | | | |
| SBIRS | … | | | | | |
| SAR | …（成像窗口/产品事件；squint 拒绝为按设计拒绝，预期出现） | | | | | |
| Fusion | … | | | | | |
| Decision | … | | | | | |

### 几何先验核对（L3）

| 先验 | 手算值 | 实测来源 | 判定 |
| --- | --- | --- | --- |
| 目标纬度 ≈ 平台纬度 + range/111 km | | | |
| EOS 距离窗 ≈ 高度/sin(俯仰角) | | | |
| 其他几何推导 | | | |

### 冒烟下限（场景 `smoke` 块）

min_key_events / min_sbirs_events / min_sar_products / min_fused_targets = …

### 结论

判定：通过 / 库问题 / 场景问题 / 预期问题（附日志证据：文件 + 行 + 周期号）

---

## 填写要点

1. **预期先于运行**：先按场景几何手算，再运行实测；实测与预期不符时先怀疑预期和
   场景，最后才是库（triage 顺序见 `triage_guide.md`）。
2. **按设计拒绝也写进预期**：SAR squint 拒绝、EOS 扫描间隙（首发现/丢失交替）、
   SBIRS 视场外——这些是"预期出现"项，不是失败。
3. **标注日志模式可观测性**：KEY 事件模式下 ESR 假设（DUP）与 SAR 持续类事件不落盘，
   别把模式门控当成事件缺失。
4. **FD 开关改变行为**：FD 开时起飞段平台向西北爬升（README 记载 hdg 293°→358°），
   EOS 探测 cycle 101 才成立；运动学回退几何干净可手算。预期表必须注明构建模式。
5. **量级给区间不给点值**（受扫描相位/积累窗口影响会抖动）。

---
---

完整填好的范例见场景归档 `examples/component_attachment/scenes/baseline_takeoff_east/baseline_takeoff_east.md`
（基线试跑样本，含实测数据与判定列）。
