# 场景预期表：sbirs_dual_sat_fix（双星交会 + 精度评估验收）

## 场景元数据

| 项 | 值 |
| --- | --- |
| 场景文件 | `examples/precision_evaluation/scenes/sbirs_dual_sat_fix/sbirs_dual_sat_fix.json` |
| 宿主 | `precision_evaluation_demo`（评估层编排，不进 `component_attachment`） |
| 场景意图 | 被测通道：精度评估（`precision_acceptance.log`）；被测行为：双星同目标双检出 → 双视线交会位置误差 + 五项 AHP。内部双 SBIRS 是评估配方（SNR 门 0.001），**不**承担 `sbirs_acceptance.log` / 宽窄视场齐套（那条用 `sbirs_wfov_nfov_handover`） |
| 构建模式 | release |
| 运行日期 | 2026-08-22 |

## 几何与门限

沿用原硬编码演示几何（GMST≈0 的儒略日，ECI≡ECEF）：

| 对象 | ECEF 位置 (m) | 速度 (m/s) | 备注 |
| --- | --- | --- | --- |
| 主星 A | (7e6, 0, 0) | 0 | 扫描起点 79.5°，周期 1 中心 ≈80.5° |
| 辅星 B | (0, 7e6, 0) | 0 | 扫描起点 351.9° |
| 目标 1 | (8e6, 6e6, 0) | (−3200, −2400, 0) | 径向下降 → 落点样本 |
| 目标 2 | (8.2e6, 5.8e6, 0) | (3266, 2309, 0) | 径向上升 → 发射点样本 |

宽/窄视场 20°/5°，扫描带 11°，`wide/narrow_min_snr_linear=0.001`。推演时域 1200 s。
姿态误差走库默认（σ=0.01°、固定 seed），指标非零且可复现。

## 验收日志开闸

精度评估写入 `precision_acceptance.log` 是库编译期开关，默认 OFF：

```bash
source scripts/activate_1q_git_bash.sh
scripts/bin/cmake --preset VisualStudio.15.0-amd64 -DONEQ_ENABLE_PRECISION_EVALUATION_LOG=ON
scripts/1q.sh build VisualStudio.15.0-amd64-release --target precision_evaluation_demo
./build/VisualStudio.15.0-amd64/Release/bin/precision_evaluation_demo.exe
# 不要从 WSL 传 /mnt/d 的 --output-dir（Windows exe 会吃掉路径）
```

默认输出：`examples/precision_evaluation/log/sbirs_dual_sat_fix/precision_acceptance.log`
（git 忽略）。demo 在首步前设置 `ONEQ_PRECISION_ACCEPTANCE_LOG_PATH`。

## 预期事件表

| 通道 | 行为 | 预期周期窗 | 预期量级 | 实测（2026-08-22，60 周期） | 判定 |
| --- | --- | --- | --- | --- | --- |
| PE | 双星交会样本 | 全程 | 每周期 `dual_sat` ≥ 1 | 60/60 周期非空；样本 119；cycle 10 为 1，其余为 2 | 通过 |
| PE | 交会位置误差 | 全程 | km 量级（评估几何 + 默认姿态噪声） | mean 1571 m，RMSE 1785 m，P95 2950 m，max 3851 m | 通过 |
| PE | 红外测角 | 全程 | <0.05° 参考 | 239 样本，RMSE 0.012° | 通过 |
| PE | AHP 五项底层分 | Summarize | 五指标均有样本、权重 0.2×5 | 底层分 (0.807, 0.849, 0.060, 0.012, 0.028)，综合分 0.351，等级 D | 通过 |

落点/发射点 RMSE 到百公里是状态误差经 1200 s 推演放大，不是交会失败。综合分 D 是参考误差标定前的演示口径（`docs/precision_evaluation/algorithms.md`），本场景门控是「有数 + 双星样本」，不是装备指标过关。

## 验收文件覆盖表（目录第 6 节）

| 项名 | 行数 | 内容 | 三分类 |
| --- | ---: | --- | --- |
| 关键精度指标（逐周期测角） | 239 | `目标键=… 方位/俯仰测角误差=…°` | 能写 |
| 关键精度指标（汇总） | 1 | `东/北/天RMSE=(1192.7,678.7,1141.0)m 距离RMSE=1784.7m 方位/俯仰RMSE=(0.0094,0.0075)° CEP50=807.9m 位置误差95%CI=[1250.0,1891.3]m 误差源贡献率=无` | 能写；贡献率按设计 `无` |
| 层次分析法 | 1 | `准则层权重=(0.200×5) 底层分=(0.807,0.849,0.060,0.012,0.028) 综合分=0.351 等级=D 贡献排序=双星>测角>速度>发射点>落点 CR=-0.000 独立多层树=无` | 能写；独立多层树按设计 `无` |

## 冒烟下限

五指标均有样本、AHP 合法、综合分 ∈ (0,1]、`dual_sat_cycles>0`。实测 **SMOKE exit=0**。

## 结论

**判定：通过**（双星交会与精度验收文件能写项已落盘）。不把本场景当作 SBIRS 宽窄视场或 `sbirs_acceptance.log` 齐套。
