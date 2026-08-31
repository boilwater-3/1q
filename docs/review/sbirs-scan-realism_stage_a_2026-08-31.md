---
Status: final
Date: 2026-08-31
Review-Baseline: `evidence/sbirs-scan-realism` @ `9cbab98a`
Authority: 非规范性记录；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准。
---

# sbirs-scan-realism：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

1、触发来源：用户核对扫描参数图解页后裁定三条（2026-08-31 会话）：扫描改往复式；跨度超地球可见范围须收敛；俯仰中心问题登记在册。
- **证据**：[evidence: docs/explainers/sbirs_scan_params_explainer.html]

2、冻结什么：锯齿单向扫描（扫完瞬时跳回起点）不符合真实扫描机构；往扫描无地球视域的空天无意义；俯仰基准 nadir 化的痛点尚未证实（GEO 下 el=0 恰为星下点俯仰）。
3、什么证据能证明：模块文档明文登记锯齿设计且测试钉住其节奏；往复式在同数值下相位/行序列与锯齿重合（影响面可控）；地球盘角度公式可算出每轨道高度的收敛上限。
4、什么证据能否定：既有验收基线依赖单方向通过时间语义且重访节奏会变；或收敛公式与门控几何冲突。
5、通过后最小改动范围：仅管线扫描推进段（腿状态+折叠反射+有效跨度）+ 快照字段 + 聚焦测试 + 模块文档；配置结构与 replay 不动。

## §1 证据矩阵

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| 往复式扫描（替代锯齿） | 锯齿瞬时回跳不符合真实扫描机构；真实机构往复摆扫（牛耕式：每次到边反向、栅格行进） | 1、锯齿单向明文登记 [evidence: docs/sbirs_sensor/algorithms.md]<br>2、相位推进为取模回绕 [evidence: src/sbirs_sensor/pipeline/SbirsPipeline.cpp] ::ScanAzimuth<br>3、EOS 同款取模回绕（全仓统一建模）[evidence: src/electro_optical_sensor/pipeline/EosPipeline.cpp] | 逐值推演既有钉住测试：TwoDimensionalRasterAdvancesRowsAndWraps / ElevationRasterGatesTargetByRow / CrossRowTargetRevisitPeriod 在 dt·rate=span 恰好过界一次的数值下，往复式相位/行序列与锯齿逐位重合（反射值=回绕值） | 实现腿状态+折叠反射推进；行进=每次过界步进一行；既有 sbirs 测试全绿（数值序列不变）；场景特征化不变 | 既有验收基线依赖单方向通过时间且数值不重合 → 只能 narrow 保留锯齿选项 | pass |
| nadir 模式扫描跨度收敛 | 扫描超出地球盘可见范围（FOV 与地球盘无交）后为无效扫描；跨度应收敛到上限 | 1、用户裁定（2026-08-31）<br>2、地球盘角半径 ρ=asin(R/|r|)，收敛上限=2(ρ+wfov_az/2)<br>3、遮挡判定同款半径 [evidence: examples/common/viz/sbirs_orbit_viewer.py] ::EARTH_RADIUS_M | 数值探针：GEO(42164km) ρ=8.69°→上限 41.38°(wfov24)；测试 LEO(7000km) ρ=65.53°→155.05°；三星场景 span=30<41.38 不触发收敛 | nadir 模式每周期 effective_span=min(span, 上限)，越过界值打一条日志；绝对模式不收敛（无锚可依，见待决说明）；场景/既有测试不受影响 | 收敛公式与 FOV 门控/行栅格几何冲突 | pass（narrow：仅 nadir 模式；绝对模式登记说明） |
| 俯仰基准 nadir 化 | 非 GEO 轨道星下点俯仰随位置漂移，手填 el 会错；GEO 下 el=0 恰为星下点俯仰，痛点未证实 | 1、GEO z≈0 → 星下点俯仰=asin(−z/|r|)≈0<br>2、上一轮冻结契约已列非目标 [evidence: docs/review/sbirs-nadir-stare-mode_stage_a_2026-08-31.md] | 公式推演：GEO 场景 el=0 免推算即正确；痛点仅在倾斜/低轨出现 | 登记 docs/common/open_questions.md，出现非 GEO 集成需求时立项 | — | defer（登记在册，不实现） |

## §2 判定汇总与待裁定问题

1、往复式（pass）：真实机构行为；既有测试数值序列在往复式下逐位重合，影响面小；scan_direction 语义变为"初始扫描方向"。
2、跨度收敛（pass/narrow）：仅 nadir 模式；公式 2(ρ+wfov_az/2)；GEO 上限 41.38°；绝对模式无法静态/运行期确定锚点，不收敛并在文档声明。
3、俯仰基准（defer/登记）：登记开放问题，再进入条件=非 GEO 轨道集成需求。
4、待用户拍板的问题：已当轮裁定完毕，见修订记录 1–3。

## §3 冻结契约（用户讨论结束后填写）

### 已证明的需求

1、扫描机构按真实往复式建模：到边反向、栅格牛耕式行进（每次过界步进一行）。
2、nadir 模式扫描跨度收敛到地球盘可见上限，超限部分不再扫描。
3、俯仰基准 nadir 化登记开放问题，本轮不实现。

### 允许范围

1、模块源码：src/sbirs_sensor/pipeline/SbirsPipeline.h（快照新字段 scan_leg_forward + 腿/有效跨度成员）；src/sbirs_sensor/pipeline/SbirsPipeline.cpp（折叠反射推进、有效跨度收敛与消费点、ApplyConfig 腿处理、收敛日志）。
2、测试：tests/unit/sbirs_sensor/sbirs_pipeline_test.cpp（往复式聚焦测试、收敛聚焦测试）。
3、文档：docs/sbirs_sensor/algorithms.md（往复式语义+收敛公式）；docs/common/open_questions.md（俯仰基准登记）。

### 明确禁止范围

1、配置结构不变：无新字段、无新枚举；scan_direction 语义微调（初始方向）不改类型。
2、replay schema/编解码不动（管线快照不在回放序列化范围）。
3、绝对 ECI 模式不做跨度收敛（无锚点；登记说明而非实现）。
4、EOS 模块不动（同款锯齿问题另行立项，不搭车）。

### 行为边界

1、输入：mission 配置语义不变（span 合法域仍 (0,360]；rate ≥ 0）。
2、输出：scan_azimuth_rad 在 span 内往复（到边反射）；行进节奏=每次过界步进一行；rate=0 时相位恒 0、腿恒初始方向（凝视行为逐位不变）。
3、收敛：nadir 模式每周期 effective_span = min(span, 2(asin(R_earth/|r_sat|)+wfov_az/2))；R_earth=6371km 与遮挡判定同款；effective_span 变化越过阈值打一条 PROJECT_LOG_INFO；相位超出新 effective_span 时归零重锚。
4、错误回退：快照恢复校验沿用（相位 ∈ [0, 配置 span) 仍恒成立，收敛只会更严）。
5、生命周期：ApplyConfig 扇区变更重锚时保留腿方向；相位归零时腿复位为初始方向。

### 验收门

1、构建：cmake --build（sbirs 相关目标）。
2、聚焦测试：往复反射（rate 常值多周期方位在 [start, start+span] 内反弹、无跳变）；收敛（GEO 位形 span=360 → 方位全程限制在星下点 ±(ρ+w/2)）；rate=0 凝视逐位不变。
3、全量：sbirs unit/replay/contract/integration 全绿；场景综合分 0.447267 逐位不变。
4、登记检查：open_questions.md 含俯仰基准条目（现状/待决/再进入条件）。

### 非目标

1、俯仰基准 nadir 化（登记项）。
2、绝对模式跨度收敛。
3、EOS 往复式改造。
4、回扫加减速/惯量建模（真实机构的端点减速不做，匀速反射）。

## 修订记录

修订 1（2026-08-31，实现期新证据）：冻结公式 `min(span, 2(ρ+wfov_az/2))` 不完备——偏移≠−span/2 时（如偏移 0）按该式裁后扇区远端仍有半程扫空天。细化为"配置扇区 ∩ 可见窗 [−(ρ+w/2), +(ρ+w/2)]"裁剪：相位原点恒为配置起点（扇区与窗交集非空时必含起点侧端点），只裁远端；交集为空（扇区整段不对地球）不收敛，归 SBIRS-OQ-5 告警登记。修订 2（2026-08-31，实现期新证据）：往复式方位映射不得翻符号——回程由相位动态（行程折叠反射）体现，相位→方位两腿同为 起点+dir·相位；若按腿翻符号会把回程画到扇区外。
修订 3（2026-08-31，用户指令）：裁定"改成往复式扫描，这才符合现实；按真实的进行设计"——锯齿→往复，作为行为替换（项目未上线，无兼容层）。
修订 4（2026-08-31，用户指令）：裁定"填 360° 不能直接扫 360°，超出后地球不在扫描区域内，该值应被收敛"——nadir 模式收敛（细化见修订 1）。
修订 5（2026-08-31，用户指令）：裁定"俯仰中心这个需要登记"——已登记 docs/common/open_questions.md SBIRS-OQ-5，不实现。

## §4 运行记录（Stage C 后填写）

1、实现范围：管线往复式推进（行程坐标 ∈ [0,2·有效跨度) 折叠反射、去/回程腿状态、牛耕式行进=每次过界步进一行）；nadir 模式跨度收敛（"配置扇区 ∩ 地球可见窗"裁剪，修订 1 细化，收敛值变化打 PROJECT_LOG_INFO）；快照新增腿方向与方位基准字段并随 Capture/Restore 往返；ApplyConfig 扇区重锚保留腿、相位归零时腿复位。同分支搭车落地 nadir 轮 completeness-review（Lane-1）修正 4 项：replay schema 字段移表尾（恢复旧缓冲 vtable 布局）、ScanPhaseForAzimuth 绝对模式逐位一致（跳过归一化）、快照补方位基准、校验补方位基准枚举合法性（新 issue code kInvalidScanAzimuthReference）。
2、验证命令与结果：
   1、`cmake --build ... 1q_lib` + 四个 sbirs 测试目标: pass。
   2、新增聚焦测试 3 项全过：PingPongSweepReversesAtSpanBoundary（6°→8°→2°→4° 反射无跳变）、PingPongSnapshotRestoresLegDirection（回程态恢复后续走 6° 非复位 10°）、NadirSpanConvergesToEarthDisk（GEO span=360 → 方位全程 ⊂ 星下点 +[0,20.69]，12 周期内必见回程）；校验测试补枚举非法用例过。
   3、全量套件：sbirs unit 263/263、replay 38/38、contract 14/14、integration 29/29、cross-domain 6/6、public API 8/8 全 pass（既有栅格/重访测试零改动通过——数值序列与往复式重合的探针结论被实测证实）。
   4、特征化：场景综合分 0.447267、dual_sat_cycles=80/80 逐位不变（rate=0 凝视下往复与收敛均不可见）。
3、权威回写去向：往复语义+收敛公式+限位/告警边界 → `docs/sbirs_sensor/algorithms.md` WFOV 搜索边界第 7/9 条与算法登记表；俯仰基准登记 → `docs/common/open_questions.md` SBIRS-OQ-5。
4、残留风险：可见窗按 el=0 赤道近似，行 el 偏离星下点俯仰时窗宽偏宽（SBIRS-OQ-5 范围）；绝对模式无告警防护（同登记）；examples 场景 main.cpp 自制装载器不解析 scan_azimuth_reference（场景保持绝对模式暂无影响，属 nadir 轮审查低危残留，随 COMMON-OQ-10 三处同构维护一并处理）。
5、后续冻结项：SBIRS-OQ-5（俯仰基准 nadir 化 + 方案1 告警）触发条件：非 GEO 轨道集成需求或 el 误配真实案例。
