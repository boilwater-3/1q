# 场景预期表：sbirs_triple_sat_fix_messages（三星 GEO + 消息机制地面站）

## 场景意图

| 项 | 值 |
| --- | --- |
| 被测通道 | SBIRS × 3 + Fusion + 精度评估（双星交会叠加） |
| 被测行为 | 赤道静止轨道三星覆盖地球盘；两两重叠区双星测角确认目标位置 |
| 验证深度 | L2 预期表（周期窗 + 通道组成）+ L3 几何先验（遮挡 / 星下点视场） |
| 构建模式 | Windows v141 release；FD 无关 |
| 日志模式 | 默认 summary / key |
| 输出目录 | `examples/log/sbirs_triple_sat_fix_messages/` |

## 几何

三颗地球静止轨道卫星（赤道、高度约 35788 km、经度间隔 120°，ECEF 速度为 0）：

| 星 | source_id | LLA（lat, lon, alt m） | 扫描扇区 | 覆盖角色 |
| --- | ---: | --- | --- | --- |
| A | 4 | (0, −25.212457, 35778820.868343) | 星下点相对 [0°,30°) | 大西洋 / 非洲西岸 |
| B | 104 | (0, −145.212457, 35778821.146480) | 星下点相对 [0°,30°) | 太平洋 |
| C | 204 | (0, 94.787543, 35778821.003790) | 星下点相对 [0°,30°) | 印度洋 / 东亚 |

从 GEO 看地球角半径约 8.7°。WFOV 24°×24°；`scan_rate=6°/s`、`scan_span=30°`
（GEO 收敛上限 2(8.69°+12°)=41.38°，不触发收敛），往复式（到边反向、牛耕式行进）。
扫描配置（2026-08-31 起）：三星均用 `scan_azimuth_reference=kNadirRelative` + 起始偏移 0，
各星独立往复扫（**无星间相位协同**——现实即如此，重叠靠几何布局）；双星交会配对窗
`precision.dual_sat_pair_window_cycles=1`（检出错开 ≤1 周期可配对，对应"按时刻融合"
口径；时间基准=周期号抽帧时刻，量测本身无时间戳属已知缺口，见
`docs/common/open_questions.md` SBIRS-OQ-6/FUSION-OQ-1）。

**两两重叠、不要求三星同视同一目标。** 120° 静止轨道对地球表面没有三星公共视场（地球遮挡）；相邻两星重叠约 42° 经度。双星测角交会已经能确认位置。

| 目标 | LLA | 可见星 | 80 s 内 |
| --- | --- | --- | --- |
| key=1 | 15°N, 85°W, 600 km | A+B（C 被地球挡住） | 仍在 A/B 视场 |
| key=2 | 10°N, 35°E, 500 km | A+C（B 被地球挡住） | 仍在 A/C 视场 |

精度评估交会只吃 A/B，因此 key=1 出 `dual_sat` 样本；key=2 只走融合通道 4+204。

## 算法边界（做不到的不要装做得到）

1. **精度评估仍是双视线 API**（`TryComputeDualLosFixM`）。第三星进融合、不进 `dual_sat_fix` 指标。这是库边界，本场景不改交会 API。
2. **融合默认沿视线起步 100 km**，GEO 真距约 36000 km。不覆写时 UKF 位置会停在卫星附近，不能当「确认位置」。本场景 JSON `fusion.track_bearing_init_range_m=3.6e7`（σ=8e6）是场景层补丁，不是库默认已按 GEO 标定。
3. **无身份探测不能靠 5° 方位门跨星关联**（不同卫星的局部方位本来就不同）。本场景靠 SBIRS 归属键直挂，所以双星能并到同一航迹。
4. **测角误差到 GEO 斜距会放大**：姿态 1-σ 0.01° × 约 39200 km ≈ 7 km 是上界口径；
   实际交会样本来自 NFOV 跟踪段（建链后每周期供数），扫描态实测交会 RMSE 约 85 m。
   早期"数公里"预期按 WFOV 单帧量测估计，已被跟踪段实测取代。
5. **NFOV 8° 若不转到目标上，离星下点约 8.5° 的目标只在 WFOV 里。** 确认位置走 WFOV 测角 + 双星交会；宽窄交接失败不算本场景失败。

## 预期事件表

| 通道 | 行为 | 周期窗 | 预期量级 | 实测（2026-08-31，扫描态） | 判定 |
| --- | --- | --- | --- | --- | --- |
| SBIRS A/B/C | 每周期交出一帧 | 1–80 | `inbox_sats=3` | 80/80 周期 `inbox_sats=3` | 通过 |
| SBIRS A | WFOV 命中 key=1 与 key=2 | 1–80 | 每周期两目标 | PE 角度样本 228 条（角度样本自周期 7 起每周期 3 条） | 通过 |
| SBIRS B | 只命中 key=1；key=2 地球遮挡 | 7–80（扫描捕获起始） | `sbirs.target_occulted` | 视图：目标 2「未检测」+ `sbirs.target_occulted` | 通过（按设计拒绝） |
| SBIRS C | 只命中 key=2；key=1 地球遮挡 | 1–80 | 同上 | 视图：目标 1「未检测」+ `sbirs.target_occulted` | 通过（按设计拒绝） |
| 融合 | key=1 双源 A+B；key=2 双源 A+C | 1–80 | `4,104` 与 `4,204` | `k1[4:10,104:10] \| k2[4:10,204:10]` | 通过 |
| 精度评估 | A/B 对 key=1 双星交会 | 7–80 | `dual_sat>0`，误差百米级 | `dual_sat_cycles=74/80`，交会 RMSE 85 m | 通过 |

冒烟：`all_metrics_sampled`、AHP 合法、综合分 0.430377 ∈ (0,1]、双星交会 74/80 周期。
缺的 6 个周期是 B 星扫描捕获 key=1 的**起始几何延迟**（期间 B 星无任何检出，
配对窗不制造量测）；建链后跟踪段每周期供数，扫描速率（6/30/60°/s 探针）不影响样本率。

落点/发射点 RMSE 约 200 km 量级：**不是交会失败**。本场景目标是中段高空平飞（ENU 水平速度），推演按弹道落点/发射点模型外推，几何对不上。位置确认看 `dual_sat_fix`，不要拿落点分当 GEO 交会指标。

每周期 stdout：`inbox_sats` 应为 3；融合航迹 key=1 带 4/104、key=2 带 4/204。

## 消息流（每周期）

```
main: app_scene.cycle = N
main: fusion->BeginCycle(world, N)
World::Step:
  各卫星 Sbirs (kMessage) → on_sbirs_frame_submitted → fusion.sbirs_inbox_
  ground_station GroundStationFusion → 重建 + 适配 + FusionEngine::Update
                                     → 从 inbox 按评估源通道挑 A/B 做 PE
```

## 构建与运行

```bash
source scripts/activate_1q_git_bash.sh
scripts/1q.sh build VisualStudio.15.0-amd64-release --target sbirs_triple_sat_fix_messages
./build/VisualStudio.15.0-amd64/Release/bin/sbirs_triple_sat_fix_messages.exe
```

输出：`examples/log/sbirs_triple_sat_fix_messages/`

## 三维可视化

场景 exe 运行时同步落盘六份几何 CSV（与验收日志同目录）：`sbirs_sats`
（卫星 ECEF + 视场角 + 扫描参数 + 星下点方位）、`sbirs_scan`（每周期每星
WFOV 扫描中心方位，供扫描条带动画）、`sbirs_truth`（真值轨迹）、
`sbirs_los`（每星逐目标视线状态/测角/SNR，取组件调试视图快照）、
`sbirs_fused`（融合航迹，LLA 后验已转 ECEF）、`sbirs_dual_fix`（双星交会
误差样本）。

```bash
python3 examples/common/viz/sbirs_orbit_viewer.py examples/log/sbirs_triple_sat_fix_messages
# 生成的 HTML 离线自包含：顶部「星下点相对方位扫描」条带（橙框 = WFOV 足印随周期移动）+
# 下方 3D 视场锥随扫描方位旋转；拖拽旋转 / 滚轮缩放 / 时间轴播放 / 图层开关。
# --check 封口测角与交会复算同库一致：
python3 examples/common/viz/sbirs_orbit_viewer.py --check examples/log/sbirs_triple_sat_fix_messages
```

## 结论

**判定：通过（位置确认成立）。** 三星 GEO 覆盖 + 两两重叠区双星交会按设计工作；地球遮挡导致的「第三星看不到」是预期。综合分被落点/发射点拖低，不代表交会失败。
