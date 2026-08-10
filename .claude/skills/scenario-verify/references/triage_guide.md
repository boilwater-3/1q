# 日志排查指南（Triage Guide）

三分类判定（库问题 / 场景设计问题 / 预期表错误）的决策树与证据要求。**每个判定必须
引用日志证据**（文件 + 行内容 + 周期号），不允许凭感觉。

## 决策树总览

```
症状（探测不上 / 事件缺失 / 量值异常 / 融合不对 / 指令没下发）
│
├─ ① 1q_library.log 有 PROJECT_LOG_ERROR/WARNING？
│     ├─ 有 → 读 message 判类别：
│     │      ├─ "按设计拒绝"类（见下表）→ 预期行为，改预期表
│     │      ├─ 业务参数类（如 SNR 低于门限）→ 场景/参数问题（②）
│     │      └─ 库内部异常类（断言/数值发散/状态机错误）→ 库问题（④）
│     └─ 无 → ②
│
├─ ② 场景几何满足被测通道边界条件（boundaries.md veto 阈值）？
│     ├─ 不满足 → 场景设计问题 → 改场景几何/参数，重跑
│     └─ 满足 → ③
│
├─ ③ 预期表自身复查（手算先验 vs 预期条目）？
│     ├─ 预期条目算错/写错 → 预期问题 → 改预期表
│     └─ 预期无误 → ④
│
└─ ④ 库问题嫌疑 → 最小复现（batch_validation 风格单会话、固定参数）
        ├─ 复现 → 库 bug → 修复 + 回归（见 SKILL.md 步骤 7）
        └─ 不复现 → 回到 ②（集成层交互引入的几何/时序问题，细化场景）
```

## 症状 → 首查日志 → 判定规则表

| 症状 | 首查日志 | 常见根因（按频率排序） | 判定 |
| --- | --- | --- | --- |
| 某通道无探测 | **先看视图行目标状态字段**（kInfo 排除诊断：`低于门限`/`不在输出`/`超出视场`等——库直接点名原因），再查 `1q_library.log` 该通道 ERROR + 边界条件 | ① 几何不满足边界（EOS 距离窗/俯仰、SBIRS 视场外、SAR squint）② 扫描相位未扫到目标（EOS 扫描间隙）③ 链路预算不足（SAR SNR、ESR 脉冲数）④ 库 bug | 视图行状态字段是判定捷径（库自述原因）；库日志说人话时先信库 |
| 事件缺失 | `integration_events.log` + 日志模式宏 | ① 该事件是 DUP 类，KEY 模式不落盘（ESR 假设、SAR 持续类、平台状态）② recorder 差分语义（如 EOS `kUpdated` 稀少是扫描断续的自然表现）③ 生命周期事件被模式门控 | 先核对 `logger/logger_modes.h` 模式与事件宏类别，别把门控当缺失 |
| 事件流"异常"（首发现/丢失交替） | 视图行（扫描方位 vs 目标方位） | 扫描断续物理（波束宽度 < 扫描步进 → 每 ~4 周期探测 1 次）→ 首发现/丢失交替是**自然表现** | 预期问题（预期写错），非库问题 |
| 量值异常（距离/方位/信噪比离谱） | 视图行量值 + 几何先验 | ① 参考系混淆（SBIRS az/el 是 ECEF 极坐标，机载是平台局部系）② 目标与平台相对运动假设错误（如 v_east 不等于巡航地速 → 目标漂移）③ 高度耦合错位（目标高度 vs 平台高度 → EOS 窗口） | 先手算先验核对，再归因 |
| 融合目标数不对 | 各通道视图 + fusion 视图/控制台 fused | ① 方位相干门限（8°）跨源残差超限 → 独立成目标 ② 异构参考系不跨源关联（SBIRS 通道独立成目标是已知语义）③ source_weights 配置 | 核对 README「天基通道」节与 FusionConfig 语义 |
| 指令没下发 | 融合置信度（控制台 fused/视图） | ① 置信度未达 `high_threat_confidence`（门限/权重/通道缺失）② 融合目标数 < 判决阈值 | 先看门限与权重，再怀疑融合/决策链库问题 |
| SAR 无产品/成像中断 | `1q_library.log` squint/slant_range 错误 + 视图行 | ① squint 超限（几何不成立——起飞/转弯段，按设计拒绝）② 斜距失配（场景中心与目标几何错位）③ SNR 低于 minimum_snr_db（RCS/功率/距离） | 逐条按错误 code 归类 |
| 库日志 ERROR 但行为正常 | `1q_library.log` | "按设计拒绝"类（squint 超限、FOV 外等）是契约行为，**预期出现** | 预期问题（预期表漏写），非 bug |

## "按设计拒绝"清单（预期出现，不是 bug）

| 通道 | 现象 | 契约依据 |
| --- | --- | --- |
| SAR | squint_angle_exceeds_limit（起飞/转弯段与几何不成立期） | 库内 squint 门控；README「SAR 侧视几何」节 |
| EOS | 首发现/丢失交替（扫描步进 > 波束宽度） | README「生命周期事件语义说明」节 |
| SBIRS | 目标在 FOV 外无探测 | 库内 FOV 20° 门控 |
| ESR | KEY 模式下假设事件不落盘 | 事件宏 DUP 门控（logger/logger_modes.h） |
| 融合 | SBIRS 独立成目标（异构参考系不跨源关联） | README「天基通道（SBIRS / SAR）场景设计」节 |

## 库问题确认后的最小复现路径

1. 从场景提取复现参数：目标几何（方位/距离/高度/速度/RCS/频率）、会话配置
   （场景覆写后的值）、周期窗；
2. 用 `tests/consumer/batch_validation` 模式写单会话复现（ArSession/EsrSession/... +
   LifecycleRecorder，跳过集成层）；若 batch_validation 已有同类 sweep，先查
   是否已被覆盖；
3. 复现成功 → 判定库问题 → 修复 + 补回归（单测或 batch_validation sequence）；
   高风险模块先走 `evidence-first-freeze-contract`；
4. 复现失败 → 回到决策树 ②（问题在集成层交互，细化场景单变量）。

## 证据格式要求

每个判定附三件套：
- **文件**：`integration_views.log` / `integration_events.log` / `1q_library.log`；
- **行内容**：引用关键字段（周期、类型、量值、问题 code）；
- **周期号**：定位到具体周期，便于复现。

例：`1q_library.log cycle 120: SAR pipeline_execution_failed: squint_angle_exceeds_limit
→ 起飞段几何不成立（按设计拒绝，预期出现），非库问题。`
