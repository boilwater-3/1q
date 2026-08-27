---
Status: implemented（2026-08-26：24 条已全部按批注裁定实施并双场景验证通过，
  逐条证据见 docs/review/acceptance_review_verification_report_2026-08-26.md；
  条23 按批注暂缓）
Date: 2026-08-26
Review-Baseline: 外部评审意见《修改.txt》（2026-08-26 经微信转达，共 24 条，原文见 §1 总表）；
  指标↔日志映射基线 `docs/review/acceptance_log_mapping_2026-08-24.md`
Authority: 评审意见逐条落实方案（现状 + 差距 + 改法）。不替代各模块 design/boundaries；
  与代码冲突时以代码为准。本文只做方案，未含已实施改动。
---

# 评审意见 24 条落实方案（2026-08-26）

## 0. 怎么读

评审在验收日志上提了 24 条意见。本文逐条给四块：**原文 → 现状（代码证据）→ 差距 → 落实方案**。
条目编号与《修改.txt》原文 1–24 一致，便于对照回复。

**总约束**（适用于全部条目，后文不再重复）：

1. **验收旁路不回灌主链**：只改验收记录层输出与必要的旁路数据结构，不改检测/调度/滤波判决主链
   （既有裁定，见 `docs/review/acceptance_item_catalog_2026-08-22.md` §0）。
2. **字符串拼接**：库内（`src/`）验收行一律 `std::string` 拼接 + `std::to_string` +
   `oneq::logging::FormatF/FormatVec3/FormatPairDeg/FormatSci`（`src/common/logging/AcceptanceText.h`）。
   **禁用** printf 风格格式化宏（VS2015/C++11 集成方无 CA_FMT_FORMAT）。
   现有五层验收记录全部如此，新增字段延续同一写法；日志文件为 GBK 编码，沿用现有 helper 可保持一致。
3. **真值分层契约**：真值只进 `precision_evaluation` 层做对照，不回写任何产品层
   （`include/1q/precision_evaluation/PrecisionEvaluationTypes.h:20-22` 注释）。涉及真值对照的条目见 §3.6、§6.1。
4. **搜索关键字即指标名**：`[验收项：…]` 内的名字不因本批改动改名（改的是行内字段，不是验收项名），
   保证既有按名搜索不受影响；`docs/review/acceptance_log_mapping_2026-08-24.md` 需随改动同步更新行内字段列。

## 1. 总表：24 条 → 模块 / 改动类型 / 方案要旨

| # | 原文意见（节选） | 模块 | 日志文件 | 改动类型 | 方案要旨 |
|---|---|---|---|---|---|
| 1 | 卫星自身定位误差打三维向量误差；找不到用传入 ECEF 随机数 | sbirs_sensor | sbirs_acceptance.log | 字段补全 | 补径向扰动 + `无` 分支按传入 ECEF 随机扰动 |
| 2 | 大幅扫描与探测缺探测概率 | sbirs_sensor | sbirs_acceptance.log | 字段新增 | 行内补 SNR + 复用雷达公共 Pd 模型 |
| 3 | 宽窄视场联合探测缺疑似目标列表 | sbirs_sensor | sbirs_acceptance.log | 字段新增 | 候选循环后补一行完整疑似列表 |
| 4 | 接力跟踪"源N"描述不清晰 | fusion | fusion_acceptance.log | 文案+数据通路 | source_id→可读名映射 + 措辞重写 |
| 5 | 协同融合句柄不明/置信度去掉/通道数含义 | fusion | fusion_acceptance.log | 字段增删 | 改逐目标行、删置信度、通道数改名限定 |
| 6 | UKF 位置估计误差=无；没有则不输出 | fusion | fusion_acceptance.log | 字段删除 | 删占位字段；误差数值落精度层行 |
| 7 | 轨迹预报时段过长（应至 10s） | target_inference | inference_acceptance.log | 参数+裁剪 | 记录层裁剪到 [0,10]s，引擎时域不动 |
| 8 | 发射点预测删发射时刻 | target_inference | inference_acceptance.log | 字段删除 | 删"预测发射时刻相对t"字段 |
| 9 | 关机点预测只留时刻+坐标 | target_inference | inference_acceptance.log | 字段裁剪 | 统一骨架两字段，未确认写"待定" |
| 10 | 落点预报误差改为关机点误差 | target_inference | inference_acceptance.log | 口径新增 | 敏度法把关机点协方差传到 API |
| 11 | 特殊事件缺识别首次探测 | sbirs_sensor | sbirs_acceptance.log | 事件新增 | 事件行补"阶段="；可选新增识别首次事件 |
| 12 | 分发状态"明文落盘未外发"暴露实现细节 | target_inference | inference_acceptance.log | 文案（或外发） | 措辞收敛；可选示例层事件外发 |
| 13 | 关键精度指标缺距离误差、脱靶量 | precision_evaluation | precision_acceptance.log | 字段新增 | 双星交会斜距误差 + 焦平面脱靶量并入汇总 |
| 14 | 天线方向图看不出主副瓣 | common/radar + rir | CSV+rir_acceptance.log | 模型修改 | 门外恒电平改连续副瓣包络 + 网格加密 |
| 15 | 航迹关联补检测结果、目标位置、检测概率 | rir | rir_acceptance.log | 字段新增 | 量测结构带 Pd，逐量测明细 |
| 16 | 航迹集合"代价"含义不明 | rir | rir_acceptance.log | 文案 | "代价"→"马氏距离²"+量纲说明 |
| 17 | 跟踪滤波位置/速度/加速度改 ECEF/LLA | rir | rir_acceptance.log | 字段新增 | 复用 coordinate 换算，行内追加 ECEF/LLA |
| 18 | 运动特征缺距离、方位、速度；类别缺失 | rir | rir_acceptance.log | 字段新增 | 行内补斜距/方位；类别见 #19 |
| 19 | "大枚举类"字样不要出现 | rir | rir_acceptance.log | 文案 | 8 处"大类枚举N(名)"→直接中文名 |
| 20 | 宽带一维像同上 | rir | rir_acceptance.log | 文案 | 同 #19，识别类型输出中文名 |
| 21 | 调度只应有跟踪、搜索两模式 | rir | rir_acceptance.log | 字段裁剪 | 删"识别=N"计数与列表段 |
| 22 | 初始化时间=暂无 | rir/sbirs + examples | 各层 + integration_events.log | 口径裁定 | 库内行引用示例层真值（方案 B） |
| 23 | 单步执行时间 39.056ms → 20ms | examples + 各层 | integration_events.log | 性能 | 先修采样口径，再分项计时定位热点 |
| 24 | 红外系统测角误差大得夸张 | sbirs_sensor | sbirs_acceptance.log | 配置+口径 | σ 配置核对 + 统计口径按星/按通道分离 |

## 2. 红外 sbirs_sensor（条 1、2、3、11、24）

### 2.1 条 1：卫星自身定位误差——三维向量误差 / 找不到用传入 ECEF 随机数

**现状**：`src/sbirs_sensor/pipeline/SbirsAcceptanceRecords.cpp:167-200`（`WriteSbirsOrbitSample`）同函数写两行：

- `[验收项：卫星自身定位误差]`（角抽样统计）；
- `[验收项：卫星ECEF三维导航定位误差]`（:196-197）：`真值ECEF=… 错误三维ECEF=… 误差向量=(dx,dy,dz)m |误差|=…m`。

误差向量**已经打印**，但只含东/北两维（`orbit_sigma` 抽样弧长映射到本地水平面），**径向恒为 0**
（:163-166 注释"径向不扰动"约定）。真值不可用分支 :199 整行写 `无`。

**差距**：① 评审要"三维"，径向缺扰动；② `无` 分支未按意见做"传入 ECEF + 随机数"回退。

**落实方案**：

1. 径向补扰动：`SbirsAcceptanceRecords.cpp:186-188` 处径向分量不再恒 0，用文件内现成 hash 版
   Box-Muller（`SampleNormal`，:70-81，新 salt 如 47U）采一维正态，σ 取 `range_fraction_sigma × 参考距离`
   （默认 0.001×4e7 m = 40 km 量级，与水平向弧长同量级）或独立配置项；口径说明 :196 同步改为"含径向"。
2. `无` 回退分支（:199）：以传入的 `input.satellite_position_ecef_m`
   （`include/1q/sbirs_sensor/session/SbirsCycleInput.h:47`，必填字段）为中心，三个分量各用独立 salt 的
   `SampleNormal` 采样叠加，输出同样的 `错误三维ECEF/误差向量` 结构，不再写 `无`。
3. 极点退化保护：`east_norm = sqrt(x²+y²)`（:179-180）极轨过极点除零，norm 近零时回退到固定单位向量。

**影响面**：仅 `SbirsAcceptanceRecords.cpp` 一个文件；不动管线主链。注意该输入字段必填且校验拒绝非有限值，
回退分支实际极少触发，属防御性补齐。

### 2.2 条 2：大幅面扫描与探测——缺探测概率

**现状**：`src/sbirs_sensor/pipeline/SbirsPipeline.cpp:1283-1289` 逐目标行：
`扫描幅宽az/el=… 目标ID=n 接收功率=…W 信号能量=…J`——无 SNR、无 Pd。SNR 本身在同循环已算出
（`ComputeSnr`，:352-374），只是没写进该行。

**差距**：缺探测概率字段。红外域库内只有 SNR 门限反解（`foundation::ComputeMaxDetectionRangeM`，
`src/sbirs_sensor/foundation/SbirsRadiometry.h:54`；门限 `wide_min_snr_linear=4.0`，`SbirsPolicyConfig.h:19-20`），
没有 IR 专用 Pd 函数。

**落实方案**：逐目标行追加 ` SNR=… 探测概率=…`：

- SNR 直接用局部变量；`snr_db = 10·log10(snr)`；
- Pd 复用雷达公共层现成模型 `oneq::common::radar::RadarEquations::ComputeDetectionProbability(snr_db, pfa, Swerling0, 1)`
  （`src/common/radar/RadarEquations.h:112`，Marcum Q 实现 :273，`StatisticalCfarDetector.cpp` 已有调用先例）；
  Pfa 由门限系数反推：高斯尾部 `Pfa = 0.5·erfc(k/√2)`，宽场 k=4、窄场 k=6，与既有门限口径自洽；
- 若评审要求红外口径不借雷达模型，备选：只补 SNR，Pd 写 SNR 与门限的比值 `SNR/门限`（无量纲裕度）。

**影响面**：`SbirsPipeline.cpp` 一处拼接；`common/radar` 头文件 include（sbirs 依赖 common 已成立）。

### 2.3 条 3：宽窄视场联合探测——缺疑似目标列表

**现状**：`SbirsPipeline.cpp:1293-1299` 宽场行每目标一行：`宽场疑似=[单个ID] 连续命中=n/m 序列确认=…`；
后续捕获过程行（:1390、:1405、:1422、:1461、:1578）同样只写单 ID。没有任何一行汇总本周期宽场疑似全列表。

完整列表就在周期局部 `std::vector<SbirsCandidate> candidates`（:848 声明、:1240 填充，
元素定义 `src/sbirs_sensor/pipeline/SbirsNfovScheduler.h:25-40`，含 target_id/snr/range_m）。

**落实方案**：候选 for 循环结束后（:1300 附近）追加一行汇总：

```text
[验收项：宽窄视场联合探测] 宽场疑似列表=[id1,id2,…] 数量=n
```

列表用 `std::to_string(id)` + `","` 拼接（先例：协同工作机制行的 `selected_text`，:1617-1643）。
`candidates` 为每星 pipeline 实例局部，双星场景无混写问题。

**影响面**：`SbirsPipeline.cpp` 一处；纯追加行。

### 2.4 条 11：特殊事件监测——缺"识别首次探测"

**现状**：事件写出 `WriteSbirsLifecycleEvents`（`SbirsAcceptanceRecords.cpp:234-264`），
事件产生 `SbirsDetectionLifecycleRecorder::Update`（`src/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.cpp:154-238`）。
现有事件只有 首次探测/跟踪中断(滑行)/跟踪中断（枚举 `SbirsDetectionLifecycleRecorder.h:23-29`）；
"首次探测"= 未检出→检出跳变（:181-191），**不区分宽场/窄场、不区分探测与识别**，
且按 episode 口径丢锁后再次检出会重复发"首次探测"。

**差距**：评审要的"识别首次探测"（首次被确认/识别）无对应事件。

**落实方案**（两级，建议同时做）：

1. 最小改动：事件行（:249-253）追加 ` 阶段=` 字段——事件结构体已带 `observation_stage`（h:59-60）只是没打印，
   映射为 `宽视场搜索/窄视场捕获/窄视场跟踪`，"首次探测"即可区分宽场首次与窄场首次；
2. 新增事件：枚举加 `kFirstRecognized`，recorder 在 `observation_stage==kNarrowFieldAcquisition && !state.recognized`
   时发出（`TargetState` 加 recognized 位），`EventName/EventLevel` 补"识别首次"/等级 1；
3. 若评审口径要求"每目标严格一次"：recorder Impl 内加 per-target seen 集合抑制重复首次探测
   （需与甲方确认是否接受丢锁后重探不再报）。

**影响面**：`SbirsDetectionLifecycleRecorder.h/.cpp`、`SbirsAcceptanceRecords.cpp`；事件枚举为公共头文件改动，
需同步 `docs/review/acceptance_log_mapping_2026-08-24.md` §5.3。

### 2.5 条 24：红外系统测角误差"大得夸张"

**现状**：两处输出均在 `WriteSbirsAngleError`（`SbirsAcceptanceRecords.cpp:203-232`）。
注入模型 `foundation::ApplyAngularErrorModel`（`src/sbirs_sensor/foundation/SbirsErrorModel.cpp:67-102`）：
az/el 误差 = `RSS(orbit, attitude, fov sigma)·N(0,1)` + 折射 + 动态滞后。σ 配置
（`SbirsPolicyConfig.h:31-38`）：`orbit_sigma_deg=0`、`attitude_sigma_deg=0.01`、`fov_sigma_deg=0`。

**排查结论（代码与实测日志核对）**：当前模型**无单位错/弧度度混用 bug**——实测偏差 az/el 最大约 ±0.03°、
会话 RMSE ≈ 0.005°，与 0.01° 1σ 完全一致。评审看到"夸张"值的可疑来源按优先级：

1. 运行场景 json 打入了非零 `orbit_sigma_deg/fov_sigma_deg`（RSS 抬升；`sbirs_wfov_nfov_handover.json:363-367`
   等场景显式置 0，`sbirs_dual_sat_fix*.json` 无 error_model 块走默认）；
2. Estimated 模式 EKF 初始暂态（`initial_position_std_m=1000` m）在近距离目标（1000 km 高度）折合 ~0.057° 尖峰；
3. **统计口径混合**：`Angle()/Orbit()` 是进程级 static 累加器（`SbirsAcceptanceRecords.cpp:60-68`），
   双星同进程时"会话RMSE"混合两颗卫星与 WFOV/NFOV 两路误差；
4. 条 1 的 `|误差|`（米级，参考距离回退 4e7 m 时数值很大）易被误读为测角误差。

**落实方案**：

1. 累加器改 per-instance：`Angle()/Orbit()` 从进程级 static 改为按（卫星实例/随机种子）键控的 map，
   净化"会话RMSE"口径；
2. "红外系统测角误差"行追加 σ 来源字段：`σ(轨道/姿态/视场)=0.000/0.010/0.000°`，评审核对量级一目了然；
3. 与甲方核对指标口径（规格 σ 是多少）：若规格更小，调 `attitude_sigma_deg` 默认值或场景 json；
   EKF 暂态尖峰属滤波收敛特性，可在回复评审时说明或加收敛段剔除（样本入统计前丢弃前 N 周期）。

**影响面**：`SbirsAcceptanceRecords.cpp`；配置默认值改动需重新裁定（关联场景 json 三份）。

## 3. 融合 fusion（条 4、5、6）

背景：融合验收行唯一写入口 `FusionEngine::Update` → `WriteFusionAcceptance`
（`src/fusion/FusionEngine.cpp:315-322`），全部行产自 `src/fusion/FusionAcceptanceRecords.cpp`。

### 3.1 条 4：接力跟踪"源N"描述不清晰

**现状**：`FusionAcceptanceRecords.cpp:252-265`：`剩余覆盖时间=488.7s(源304) 接力计划=源304预计…离开视场
交接指令=源304→源4@T+488.7s`。`源N` 即 `ChannelMeasurement::source_id`（场景 json `satellites[].source_id`，
本场景 4/104/204/304）。可读名称**断了链**：json 里有 `"id": "A"/"B"/…`，main.cpp 读入后只打 stdout 与实体名，
不进 `SbirsFrameSubmittedEvent`（只有数字 source_id），也不进融合库。

**落实方案**：

1. `FusionConfig`（`include/1q/fusion/FusionConfig.h`）加 `std::map<std::uint32_t, std::string> source_names`
   （名字是装配配置不是真值，不破分层契约）；场景 main 装配前由 `satellites[]` 填入；
2. `FusionAcceptanceRecords.cpp` 新增 `SourceLabel(id)` 帮手：有映射写 `卫星A(源4)`，无映射回退 `源4`；
   替换 :253-259 三处与 `ChannelNames()`（:63-76）的裸 `"源"+std::to_string(id)`；
3. 措辞同步重写：`交接指令=源304(剩余488.7s离场)→源4 接管`，去掉 `@T+` 缩写。

**影响面**：`FusionConfig.h`（公共头，加字段向后兼容）、场景 main 装配代码、`FusionAcceptanceRecords.cpp`；
无测试断言验收字符串，风险低。

### 3.2 条 5：协同探测信息融合——句柄不明 / 置信度去掉 / 通道数是指

**现状**：`FusionAcceptanceRecords.cpp:287-291`，每周期只发**一行**且只用 `tracks.front()`：

```text
参与通道=源4+源104+源204+源304 通道数=4 融合置信度=4.000 融合目标数=2
```

- 句柄恒定：是——首航迹的通道集合整场不变，评审看到的"所有都是"属实；
- `通道数` = 首航迹滑窗内有量测的源通道个数，**不是**系统通道总数，名字误导；
- `融合置信度` = 首航迹的 `FusedTarget::confidence`，评审要求去掉；
- 各目标各自的参与通道（`track.channels`）只进了 stdout，没进验收日志——"跟踪目标的协同融合信息缺失"属实。

**落实方案**：:287-291 移入 per-track 循环，每航迹一行：

```text
[验收项：协同探测信息融合] 目标键=k 参与通道=卫星A(源4)+卫星B(源104) 参与通道数=2
```

删 `融合置信度`；`通道数` 改名 `参与通道数`（该目标滑窗内有量测的源个数）；句柄随目标自然可区分
（配合条 4 的 `SourceLabel`）。`融合目标数` 保留在首行或并入多目标跟踪行，二选一（建议保留一行汇总）。

**影响面**：`FusionAcceptanceRecords.cpp` 单文件。

### 3.3 条 6：UKF 位置估计误差=无——没有则不输出

**现状**：`FusionAcceptanceRecords.cpp:281` 无条件硬编码 `位置估计误差=无`，函数内无真值参数。
估计−真值**已在精度层算出**：`PrecisionEvaluationSession.cpp:320-348` 按 key 匹配融合航迹与真值，
`VelocityErrorSample.position_error_m`（`PrecisionEvaluationTypes.h:113-118`，注释明确"供参考，不进 AHP"）现成。

**分层约束**：真值只进精度评估层（总则 3），**不能**把真值塞进融合库算误差。

**落实方案**（并行两件）：

1. 删 `FusionAcceptanceRecords.cpp:281` 该字段（"没有对照就不输出"）；
2. 误差数值落精度层：`WritePrecisionKeyMetrics` 逐周期行（`src/precision_evaluation/PrecisionAcceptanceRecords.cpp`）
   追加 `航迹位置误差=…m`（直接读 `cycle_result.velocity[i].position_error_m`，数据现成）；
   `docs/review/acceptance_log_mapping_2026-08-24.md` §3.10"估计误差指标"行的指引同步改为指向精度层该字段。

**影响面**：`FusionAcceptanceRecords.cpp` 一行删除 + 精度层行拼接 + 映射文档。

## 4. 推演 target_inference（条 7、8、9、10、12）

背景：全部行产自 `src/target_inference/InferenceAcceptanceRecords.cpp`；引擎 `TargetInferenceEngine.cpp`。

### 4.1 条 7：目标轨迹预报——预报时段过长

**现状**：`InferenceAcceptanceRecords.cpp:285` 写 `预报时段=[h0,h1]s`，h1 来自航路点表尾
（引擎外推 `while (t < config_.prediction_horizon_sec)`，`TargetInferenceEngine.cpp:196`；默认 300s，
场景覆写 1200s；撞地即截断）。实测行 `预报时段=[1.0,640.0]s` / `[1.0,1200.0]s`。

**差距**：评审要"当前周期时刻到 10 秒"。

**落实方案**（推荐 1，备选 2；**不能**把 `prediction_horizon_sec` 直接改成 10——那会杀掉落点解算
（样例落点 640s 才解出）并破坏 precision 层 1200s 依赖）：

1. 记录层裁剪（最小改动）：:279-285 只保留 `time_offset_sec <= 10.0` 的航路点，预报时段写 `[0,10]s`
   （下界强制 0——首个点是 1.0s，当前点可在行内补当前 LLA）；
2. 引擎参数化（更干净）：`TargetInferenceConfig` 新增 `forecast_waypoint_horizon_sec{10.0}`，
   航路点发射处（`TargetInferenceEngine.cpp:226-235`）加门，积分继续跑到落地供落点/误差用；
   顺带消除当前每航迹产出 121 个航路点、记录层逐点拼接的浪费。

### 4.2 条 8：发射点预测——删发射时刻

**现状**：`InferenceAcceptanceRecords.cpp:325` `预测发射时刻相对t=…s` 为该行首字段。

**落实方案**：删 :325，:326 `地理位置=` 提为行首。API 字段 `launch_time_offset_sec`（`InferenceResult.h:42`）
**保留不动**——关机点 kBeforeWindow 分支（:252-256）还在用，该分支按条 9 裁剪后引用自然消失。

### 4.3 条 9：关机点预测——只留关机点时刻 + 坐标

**现状**：`InferenceAcceptanceRecords.cpp:221-264` 按状态分支输出，仅"已确认"分支
（:227-242）带 `关机时刻=…s 关机点经纬高=…`，还多 2 个字段（当时速度、机械能峰值）；
观测中/助推中/早于跟踪起点/窗口外 四个分支两字段皆缺（只有能量峰值等）。

**落实方案**：统一骨架 `目标键=N 关机点时刻=X.Xs 关机点经纬高=(…)`：

- kConfirmed 复用 :229/:233-239 写法，删 :240-241 与各行 `状态=`；
- 未确认分支写 `关机点时刻=待定 关机点经纬高=待定`（状态机暂态峰值不宜当关机点输出）；
- 无航迹整行 `暂无`（:203）保留。

**影响面**：记录层单文件；状态机本体（`UpdateBurnoutTracker`）不动，单测（相位逻辑）不受影响。
若评审还需要区分"为什么待定"，行内可加 `原因=`，待确认（见 §8）。

### 4.4 条 10：落点预报误差改为关机点误差

**现状**：`InferenceAcceptanceRecords.cpp:308` `误差1σ=` 取 `traj.impact_position_sigma_m`——
落点协方差前向敏度法传播（引擎 :344-387，6 扰动重推到落点时刻，对角简化）。
**关机点误差当前未算**：`BurnoutTrackerState`（记录层静态表，Records.cpp:41-44）只有 position/speed/time，
无协方差，引擎不知道关机点。

**落实方案**（推荐 1）：

1. 引擎内计算并出 API：关机点判定复用于 `Infer()`（状态机改引擎成员，或记录层回传确认的
   `关机时刻偏移 = state.time_sec − track.sim_time_sec`）；对已确认航迹用同一套敏度框架
   （`Rk4Step` 支持负 dt，:57-58，回推到关机时刻无障碍）传播当前协方差 `track.covariance_ecef`，
   `TrajectoryPrediction` 新增 `burnout_position_sigma_m`（仿 :44 launch 字段先例）；
   记录层 :308 改 `关机点误差1σ=…m`；
2. 轻量版：引擎公开 `double PositionSigmaAt(const InferenceTrackState&, double t_offset_sec)`，
   记录层确认分支调用写行。

**注意**：轨迹预报行的"误差椭圆"（:290-291）与落点 σ 同源，口径是否跟随改（关机点椭圆 vs 落点椭圆）
需与甲方确认（见 §8）；`InferenceResult.h:39` 注释（2026-08-20"椭圆参数不输出"裁定）需同步。

### 4.5 条 12：分发状态"明文落盘未外发"

**现状**：`InferenceAcceptanceRecords.cpp:316` `分发状态=明文落盘未外发`；:312-315 `标准化封装=明文:[落点预报|…]`。
:310-311 注释说明这是 2026-08-22 甲方批注"写明文"的结果——库内无标准编解码与发布订阅，如实标注。
全库确无 zmq/mqtt/socket/消息总线；示例层仅 SBIRS→地面站帧有一套事件机制
（`examples/core/events.h` + Boost.Signals2，`sbirs_dual_sat_fix_messages` 场景），推演结果无事件分发。

**落实方案**（按力度递增，建议先 1、视排期做 2）：

1. 措辞收敛（最小，不改架构）：`标准化封装=明文:[…]` → `标准化封装=[落点预报|落点=…|1σ=…|置信度=0.68]`
   （去掉"明文:"前缀）；`分发状态=明文落盘未外发` → `分发状态=已封装待分发`。去掉实现细节，内容仍属实；
2. 真外发（示例/集成层，库内不做）：仿 `sbirs_dual_sat_fix_messages` 模式，`events.h` 增
   `ImpactForecastEvent`（周期、目标键、落点 LLA、σ、置信度），运行引擎的组件每周期 publish，
   验收行改 `分发状态=已发布(事件 on_impact_forecast_published)`。事件机制按 `events.h` 头注释定位为
   集成契约，落 examples 侧，不进 `src/target_inference`——回复评审时说明该边界。

**影响面**：方案 1 记录层单文件；方案 2 涉及 `examples/core/events.h`、对应场景组件与 main。

## 5. 雷达 remote_identification_radar（条 14–21）

背景：全部行产自 `src/remote_identification_radar/runtime/RirAcceptanceRecords.cpp`（746 行）。

### 5.1 条 14：天线方向图看不出主副瓣

**现状**：增益模型单源 `src/common/radar/AntennaPatternRuntime.h`：

- `EvaluateAntennaPattern`（L248-273）：主瓣内 `peak − 高斯锥削 − 扫描损耗`；**主瓣外直接
  `peak + max_sidelobe_level_db(−20dB)` 恒定电平**（L271），后瓣恒 −35dB——门外是平的，无副瓣结构；
- sinc 模式（L190-212）能产生 sinc² 副瓣但需 `antenna_length_m/width_m > 0`，默认 0（`RirHardwareConfig.h:103-106`）
  未启用；
- CSV 网格（`RirAcceptanceRecords.cpp:713-715`）：az/el −90..90° **步进 2.0°**，默认波束宽 4°，
  主瓣内只有 ~2×2 个采样点，主瓣形状都画不出来，更别说副瓣。

**落实方案**：

1. 副瓣结构：`EvaluateAntennaPattern` L266-272 门外段替换为连续方向图——`(sinc(u)/u)²` 幂次
   （沿用 L190-212 已有 sinc 核）或余弦幂锥削 `cos^p` 连续延拓，再以 `max_sidelobe_level_db`
   作副瓣包络钳制（`min(计算值, peak+sl_level)`，保持配置语义）；后瓣仍用 `backlobe_level_db`；
2. CSV 网格自适应：`TryExportRirAntennaPatternCsv`（:699-726）步进改 `max(0.1, bw/16)`，
   保证主瓣 ≥8 点、前几个副瓣可辨；必要时 az 范围收窄到 ±3bw；
3. 汇总行（:246-255，`波束宽度az/el=(4.000,4.000)°`）口径已自洽（半功率宽），不动。

**影响面**：`common/radar/AntennaPatternRuntime.h` 是单源（RIR/SAR 等共用）——改门外增益会影响所有
调用方的门旁增益值（检测链 SNR 抖动），需跑回归；保守路线是只在 CSV 导出处单独用含副瓣公式，
主链不动（旁路线见 §8 待确认）。

### 5.2 条 15：航迹关联——补检测结果、目标位置、检测概率

**现状**：`RirAcceptanceRecords.cpp:406-427`（`WriteRirAssociation`）：
`新量测=N 配到已有航迹=M 未配上=K 命中：航迹A←量测B 代价=…`——只有计数与命中对，
无逐量测的位置/是否命中/该量测 Pd。

数据可得性：量测位置在 `RirAssociationResult::measurements`（`RirTrackTypes.h:58-72`，已传入本函数）；
是否命中 = `matched_existing_track`（关联后回填，`RirTrackAssociator.cpp:195-204`）；
**Pd 当前不进关联结构**——在检测链 `RirController::TryBuildMeasurement`（`RirController.cpp:574-576`）产出，
只写了 `[验收项：统计检测概率]` 行。

**落实方案**：`RirTrackMeasurement`（`RirTrackTypes.h:58`）加 `float detection_pd{0.0f}`；
`RirController.cpp:656` 前回填 `built.detection_pd = detection.detection_prob`；
`WriteRirAssociation` L414-423 命中段扩为逐量测枚举：

```text
量测i(目标ID=…,位置ENU=(x,y,z)m,Pd=0.86) → 航迹A 命中 / 未配上
```

**影响面**：`RirTrackTypes.h`（内部结构加字段）、`RirController.cpp` 回填一行、记录层拼接；旁路数据，不进关联判决。

### 5.3 条 16："代价"含义不明

**现状**：`RirAcceptanceRecords.cpp:422` `代价=0.1234`；计算 `ComputeSquaredMahalanobisDistance`
（`src/remote_identification_radar/tracking/RirTrackAssociator.cpp:47-59`，d²=νᵀS⁻¹ν，无量纲；
波门 `gate_threshold = distance_gate_sigma²`，`RirController.cpp:242-243`）。

**落实方案**：纯文案——`代价=` 改 `马氏距离²=`，行尾一次性追加量纲说明
`（d²=νᵀS⁻¹ν，无量纲，波门=9）`，门限值从 `RirAssociationConfig` 传入。

### 5.4 条 17：跟踪滤波位置/速度/加速度改 ECEF/LLA

**现状**：`RirAcceptanceRecords.cpp:521-524` `[验收项：目标位置速度加速度估计]`：
`航迹=N 当前ENU m=(…) 下一时刻预测ENU m=(…)`（预测 dt 硬编码 1.0，:472-475）——只有位置，
速度/加速度在同函数 `典型/再入目标跟踪`（:494-499）与 `跟踪滤波`（:504-528）行，全部 ENU。

现成换算：`include/1q/coordinate/position_transform.h`（`TryEcefToLla` L75、`TryEnuToEcef` L166）、
`velocity_transform.h`（`TryEnuToEcefVelocity` L103）；锚点 `RirCycleInput::platform_position`（ECEF）
现成，`RirController::RunCycle` 已在 :696-699 用它换过平台 LLA。

**落实方案**：`WriteRirTrackAndId` 签名（`RirAcceptanceRecords.h:95-100`）增 platform ECEF/LLA 参数
（调用处 `RirController.cpp:991` 传入）；涉及行（位置速度加速度估计 / 典型再入跟踪 / 跟踪滤波）各追加：

- 位置：`ECEF=(x,y,z)m LLA=(lat,lon,alt)`（ENU→ECEF→LLA 两跳，工具全有）；
- 速度：`ECEF速度=(vx,vy,vz)m/s`（`TryEnuToEcefVelocity`）；
- 加速度：无现成函数，用 ENU→ECEF 旋转矩阵乘加速度向量（原点不位移，与速度同一旋转），
  可在 `velocity_transform.h` 加 `TryEnuToEcefVector` 小工具或记录层内联展开。

ENU 字段是否保留：建议**保留**（兼容既有读者），ECEF/LLA 追加在后；评审明确要替换时再删（§8 待确认）。

### 5.5 条 18：运动特征处理——缺距离、方位、速度；目标类别缺失

**现状**：`RirAcceptanceRecords.cpp:558-566`：`航迹=N 速度=… 高度=… 近似直线=… 目标类别=大类枚举N(名)`。

- 距离/方位**可得未写**：`features`（`RirFeatureMeasurementRecord`）自带 `range_m/look_az_deg/look_el_deg`
  （`RirFeatureMeasurementTypes.h:99-102`）；函数内 `polar`（斜距/方位/俯仰）已算好只用于其它行（:477）；
- 速度字段已有（评审所指"缺速度"可能是缺 ECEF/LLA 口径的速度——由条 17 一并覆盖）；
- "目标类别缺失"：真实输出是 `大类枚举-1(无)`——识别未确认时 `target_category=kUnknown`
  （确认条件 `best_score >= acceptance_score` 且滑窗聚合有效，`RecognitionTracker.cpp:347-357`；
  运动特征本身要求航迹已确认，`MotionFeatureExtractor.cpp:24-26`）。

**落实方案**：行内追加 ` 斜距=…m 方位/俯仰=…°`（直接用 features 字段）；类别显示归并条 19
（`大类枚举-1(无)` → `未识别`）；若评审要求类别早出（accumulating 阶段即输出当前最优候选），
改 `RecognitionTracker` 分支——属识别主链参数，需单独裁定（§8）。

### 5.6 条 19 + 条 20："大枚举类"字样不要出现

**现状**：8 处输出 `大类枚举N(中文名)` / `识别大类枚举N(…)`：`RirAcceptanceRecords.cpp:486、549、564、571、
578、585、617、620`。中文映射 `CategoryName`（:45-65）已完备：弹道目标/临近空间目标/其它/未知/战斗机/
轰炸机/导弹，nullptr → 无。

**落实方案**：纯文案——8 处全部删"大类枚举N()"外壳，直接输出 `目标类别=战斗机`、`大类=未知`、
`识别类型=导弹`；`category=-1` 时输出 `未识别`（`CategoryName` default 分支"无"改"未识别"或调用处特判）。

**影响面**：`RirAcceptanceRecords.cpp` 单文件 8 处字面量。

### 5.7 条 21：调度策略——只应有跟踪、搜索两个模式

**现状**：调度器实际只有"扫描波位推进 vs 指定目标驻留"两态（`RirSession.cpp:330-360`；
工作模式枚举 `RirWorkMode` 本就只有 kStby/kIdentify 两态，`RirMissionConfig.h:27-31`）。
`[验收项：各类事件的实际执行列表]`（`RirAcceptanceRecords.cpp:638-641`）写 `[搜索×Ns,跟踪×Nt,识别×Ni]`——
"识别×N"是识别参与计数（`RirController.cpp:922-933`：`ident_count = participating_track_count`），
不是独立驻留类型。三分类文案误导了评审。

**落实方案**（推荐 A）：

- A：`WriteRirSchedule` 签名去掉 `ident_count`（`RirAcceptanceRecords.h:102-105` 同步），
  行内删 `识别=N` 与 `[…,识别×Ni]` 段；`RirController.cpp:929-932` 调用改两计数；
- B（备选）：识别并入跟踪，`跟踪/识别=N` 合并计数。

映射文档 §3.11 调度策略行同步更新。

## 6. 精度评估 precision_evaluation（条 13）

### 6.1 条 13：关键精度指标——缺与目标的距离误差、脱靶量

**现状**：`WritePrecisionKeyMetrics`（`src/precision_evaluation/PrecisionAcceptanceRecords.cpp:56-91`）：
`东/北/天RMSE、距离RMSE、方位/俯仰RMSE、合成RMSE、CEP50、95%CI`。注意现有"距离RMSE"是
**双星交会定位三维误差 RMSE**，不是传感器—目标斜距误差——评审所指即此缺口。

**数据通路核查**：

- 斜距误差：真值斜距其实已算出但被丢弃（`TryComputeTruthAzElRad`，`PrecisionEvaluationSession.cpp:77`）；
  量测侧 SBIRS 无源测角无真实测距，**可得的真实口径是双星交会解 `fix_position` 到卫星的距离 vs 真值到卫星的距离**
  （交会样本落盘前就在手上，:292-303）；
- 脱靶量：库内有现成计算但只在 SBIRS 层日志文本里（焦平面映射 `SbirsFocalPlaneOffset`，
  `src/sbirs_sensor/foundation/SbirsGeometry.cpp:184-194`；消费点 `SbirsPipeline.cpp:1096-1113`
  拼进 `[验收项：窄视场跟踪探测]` 行），未进精度层、无 RMSE 汇总。

**落实方案**：

1. 斜距误差：`DualSatFixSample`（`PrecisionEvaluationTypes.h:98-103`）加 `slant_range_error_m`
   （= `|sat_a − fix| − |sat_a − truth|`，`DistanceM` 现成）；Impl 加序列累加；
   `WritePrecisionKeyMetrics` 签名扩一个 `const std::vector<double>&`，:79 后插 ` 斜距RMSE=…m`；
2. 脱靶量：SBIRS 层把已算好的焦平面偏移写进 `SbirsDetectionAttributionRecord`
   （`SbirsOutputTypes.h:93` 已有 `nfov_pointing_error_deg` 先例，新增脱靶量米/像素字段，
   填在 `SbirsPipeline.cpp:1096-1100` 并搬到日志开关块外）；精度层 Step 遍历归属记录累加，
   Summarize 后同行追加 ` 脱靶量RMSE=…m`。

**影响面**：`PrecisionEvaluationTypes.h`（样本结构加字段）、`SbirsOutputTypes.h`（归属记录加字段）、
两个 Session/Records 文件；均为旁路数据，不动评估主链。

## 7. 性能与计时（条 22、23）

### 7.1 条 22：初始化时间=暂无

**现状**：库内占位两处——RIR `RirAcceptanceRecords.cpp:646-660`（`WriteRirOncePerSession`，进程级
static 只写首次）与 SBIRS `SbirsAcceptanceRecords.cpp:267-278`，均写 `暂无`。
示例层**已有真实墙钟**且带同名验收关键字：`examples/app/runner.cpp:168-184`
（`LogAcceptanceMs("初始化时间","RIR",…)`、`单个模型加载时间 = LastRecognitionDatabaseLoadMs()`——
库内其实已有识别库加载计时器只是没接到占位行；`acceptance_timing.h:22-26` 行格式
`[验收项：…] 验收内容：…ms 模块=…`）。同一验收项两个文件一边"暂无"一边有真值，评审看层内文件只见"暂无"。

**落实方案**（推荐 B，与现行"库内占位保持"裁定一致）：

- B：库内占位行文案改为 `见 integration_events.log [验收项：初始化时间]（模块=RIR）`；
  同时补齐示例层缺口——`sbirs_dual_sat_fix` main 的 FusionEngine/PrecisionEvaluationSession 构造
  加 `SteadyElapsedMs` + `LogAcceptanceMs("初始化时间","Fusion"/"Precision",…)`；
  runner 补 SAR/ESR/AR/EOS/Inference 的 Create 计时；`customer_integration_verification.md:206` 检查单与
  映射文档 §4 末行扩模块清单；
- A（备选，回填库内）：会话 Create 暴露墙钟（照 `LastRecognitionDatabaseLoadMs()` 先例），
  占位行换真值——跨层传值（时钟归 example 所有）且与既有裁定冲突，需重新裁定。

### 7.2 条 23：单步执行时间 39.056ms → 20ms

**现状核查**：

- "39.056ms" 全仓无命中（代码/文档/存档日志）；最近实测是 `acceptance_review_2026-08-23.zip` 内
  `单步执行时间 5.122ms 模块=RIR`（首周期）。39.056 应为评审方自测，**最可能是 SBIRS 首步**
  （首周期含一次性分配/预热，采样方法本身高估）；
- 现有测量只两处且**只测首个周期**：RIR `rir_sensor_component.cpp:201-207`、
  SBIRS `sbirs_sensor_component.cpp:344-350`（一次性标志）；整场景 `world.Step(dt)`
  （`runner.cpp:400`）裸调无计时；ESR/ECM/AR/EOS/SAR/Fusion/Inference/Threat 全无计时；
- 库内唯一分项计时：SAR RDA 聚焦（`SarRda.cpp:255-295`，DEBUG 级默认不落盘）；
  `acceptance_metric_status_2026-08-22.md:101` 明确"库内无逐层计时"。

**落实方案**（先口径后优化，分三步）：

1. **修采样口径**：首周期单样本 → warmup 后 N 周期 mean/P95（照搬
   `tests/performance/cross_domain/rf_interference_performance_test.cpp:224-269` 模式），
   否则 20ms 目标无法判定——首步 39ms 与稳态可能差一个量级；
2. **分项定位**：RIR/SBIRS 组件的两行计时模式复制到其余组件 Step（同一 `LogAcceptanceMs`，模块名区分）；
   `runner.cpp:400` 整场 `world.Step` 加总计时行，得"各模块之和 vs 整场"差值（≈组件间适配/事件发布开销）；
3. **库内热点**（按代码结构排序的候选）：SBIRS 逐目标 LOS/SNR/地球交会投影 + NFOV EKF
   （`SbirsPipeline.cpp` 1805 行逐周期逐目标）；RIR 前端逐发射源 incident-link 循环
   （`RirRfFrontEndResolver.cpp:37-40`，O(发射源数)×驻留）；**验收日志字符串构建本身在被计时的 Step 内**
   （rir_acceptance.log ≈20KB/周期）；融合逐航迹 UKF（`FusionEngine.cpp:575/606/639`，sbirs demo 强制开）。
   优化手段按结构：incident-link 与逐航迹滤波可按目标/航迹并行；验收日志可抽样降频输出
   （examples 视图已有三密度模式先例）；SAR 聚焦 FFT 分段摊到积累周期。

**影响面**：第 1、2 步全在 examples 层（零库改动）；第 3 步逐项另立任务单，每项优化后跑
`scripts/1q.sh test VisualStudio.15.0-amd64-release` 回归。

## 8. 待评审/甲方确认的口径与建议执行顺序

**需确认口径**（不阻塞其余条目）：

| 条目 | 待确认 | 建议默认 |
|---|---|---|
| 1 | 径向扰动 σ 口径（range_fraction×参考距离 vs 独立配置） | range_fraction×参考距离 |
| 2 | 红外 Pd 借用雷达 Marcum Q 模型是否可接受 | 接受（附 SNR 字段可自查） |
| 9 | 未确认分支"待定"是否需要附原因 | 只写待定 |
| 10 | 轨迹预报行误差椭圆是否随改关机点口径 | 保持落点口径，仅发布行改关机点误差 |
| 12 | 是否要求真外发（方案 2 排期） | 先措辞收敛 |
| 14 | 主链门外增益跟随改（回归成本）vs 仅 CSV 导出改 | 主链与 CSV 一起改，跑回归 |
| 17 | ENU 字段保留还是替换 | 保留 + 追加 ECEF/LLA |
| 18 | accumulating 阶段是否输出候选类别（动识别主链） | 不动主链，未确认显示"未识别" |
| 22 | 库内回填（A）vs 指引示例层（B） | B |
| 23 | 20ms 目标的采样口径（稳态 P95 vs 首步） | 稳态 P95 |

**建议执行批次**：

1. **文案/字段裁剪批**（纯记录层，零风险，一次提测）：条 6、8、9、12-1、16、19、20、21、22-B；
2. **字段追加批**（记录层 + 少量旁路结构）：条 1、2、3、5、11-1、13、15、17、18；
3. **模型/架构批**（需回归或排期）：条 4（source_names 通路）、7（引擎参数化）、10（关机点协方差）、
   14（方向图模型+CSV）、24（累加器口径+σ 来源）；
4. **性能批**（独立任务单）：条 23 三步走，11-2（识别首次事件）并入批次 2 或 3 视排期。

## 9. 测试与回归影响汇总

- 现有单测**不断言任何验收字符串**（fusion_relay_prediction_test 只测接力数学函数；inference 测试只测
  `UpdateBurnoutTracker` 相位逻辑）——批次 1、2 全部改动不破坏现有测试；
- 批次 3 中条 14（`common/radar/AntennaPatternRuntime.h` 单源）与条 24（配置默认值）影响面跨模块，
  改后跑全量 `scripts/1q.sh test VisualStudio.15.0-amd64-release`；
- 公共头改动清单：`FusionConfig.h`（source_names）、`RirTrackTypes.h`（detection_pd）、
  `SbirsOutputTypes.h`（归属记录脱靶量）、`PrecisionEvaluationTypes.h`（斜距误差）、
  `SbirsDetectionLifecycleRecorder.h`（事件枚举，若做 11-2）、`TargetInferenceConfig.h`/`InferenceResult.h`
  （若做 7-2/10-1）——全部为加字段/加枚举，向后兼容；
- 每批落地后同步更新 `docs/review/acceptance_log_mapping_2026-08-24.md` 行内字段列
  （§5.1/§5.2/§5.4/§5.5/§3.9–§3.11/§4/§5.6/§5.7 对应小节）。

[evidence: src/sbirs_sensor/pipeline/SbirsAcceptanceRecords.cpp]
[evidence: src/sbirs_sensor/pipeline/SbirsPipeline.cpp]
[evidence: src/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.cpp]
[evidence: src/sbirs_sensor/foundation/SbirsErrorModel.cpp]
[evidence: src/fusion/FusionAcceptanceRecords.cpp]
[evidence: src/fusion/FusionEngine.cpp]
[evidence: src/target_inference/InferenceAcceptanceRecords.cpp]
[evidence: src/target_inference/TargetInferenceEngine.cpp]
[evidence: src/remote_identification_radar/runtime/RirAcceptanceRecords.cpp]
[evidence: src/remote_identification_radar/tracking/RirTrackAssociator.cpp]
[evidence: src/common/radar/AntennaPatternRuntime.h]
[evidence: src/common/radar/RadarEquations.h]
[evidence: src/precision_evaluation/PrecisionAcceptanceRecords.cpp]
[evidence: src/precision_evaluation/PrecisionEvaluationSession.cpp]
[evidence: examples/app/runner.cpp]
[evidence: examples/logger/acceptance_timing.h]
[evidence: docs/review/acceptance_log_mapping_2026-08-24.md]
