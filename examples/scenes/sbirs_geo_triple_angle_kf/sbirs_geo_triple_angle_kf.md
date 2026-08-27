# 场景预期表：sbirs_geo_triple_angle_kf（三星 GEO + 角度域线性 KF）

## 场景意图

| 项 | 值 |
| --- | --- |
| 被测通道 | SBIRS × 3 + Fusion + 精度评估（双星交会叠加） |
| 被测行为 | 与 `sbirs_dual_sat_fix_messages` 同几何；Estimated 后端改为实验 `kAngleCvKf`，检查滤波方位/俯仰、变化率、角度航迹稳定性 |
| 验证深度 | L2 预期表 + L3 与原场景（默认 `kEkf`）对照 |
| 构建模式 | Windows v141 release；FD 无关 |
| 日志模式 | 默认 summary / key |
| 输出目录 | `examples/log/sbirs_geo_triple_angle_kf/` |
| 运行日期 | 2026-08-27 |

## 与原场景的唯一算法差

几何、三星位置、目标、WFOV/NFOV、消息地面站、融合起步斜距全部同 `sbirs_dual_sat_fix_messages`。

| 项 | 原场景 | 本场景 |
| --- | --- | --- |
| Estimated 后端 | 默认 `kEkf`（6 维 ECI CV） | 显式 `kAngleCvKf`（4 维视线角 CV） |
| `process_noise_diff_coeff` | 默认 1.0（m²/s³） | `1e-8`（rad²/s³） |
| 变化率 | 不落公开检测记录 | 验收行「目标角度状态估计」+ 滤波器内部状态（不改 `SbirsDetectionRecord`） |

原场景实测已进入 NFOV 估计跟踪（`窄视场跟踪探测` 316 条 / 80 周期），对照是两种 Estimated 后端，不是「无滤波 vs 滤波」。

## 几何

同原场景：赤道静止轨道三星、120° 经度间隔、WFOV 24°×24° 星下点凝视。key=1 可见 A+B，key=2 可见 A+C。精度评估交会只吃 A/B。

## 算法边界

1. 精度评估仍是双视线 API；第三星进融合、不进 `dual_sat_fix`。
2. 融合 `track_bearing_init_range_m=3.6e7` 是场景层 GEO 补丁。
3. `kAngleCvKf` 后验不得解释为三维位置/速度；位置确认仍看双星交会。
4. 变化率不进公开 `SbirsDetectionRecord`。场景 CSV `angle_track.csv` 只含滤波后 az/el；变化率看 `sbirs_acceptance.log` 的「目标角度状态估计」。
5. 原场景 EKF 用场景真值三维初始化均值；本场景角度 KF 只用当前测角、变化率置 0。对照时 EKF 带真值起步优势。

## 预期事件表

| 通道 | 行为 | 周期窗 | 预期量级 | 实测（2026-08-27） | 判定 |
| --- | --- | --- | --- | --- | --- |
| SBIRS A/B/C | 每周期交出一帧 | 1–80 | `inbox_sats=3` | 80/80 `inbox_sats=3` | 通过 |
| SBIRS | NFOV Estimated + kAngleCvKf | 捕获后 | `目标角度状态估计` 含方位/俯仰变化率 | 316 条；方位变化率 RMSE 0.00062°/s，俯仰 0.00240°/s | 通过 |
| SBIRS | 滤波角度航迹连续 | 捕获后 | `angle_track.csv` 每可见星-目标一行 | 320 行（4 条航迹 × 80 周期） | 通过 |
| 融合 | key=1 双源 A+B；key=2 双源 A+C | 1–80 | `4,104` 与 `4,204` | `k1[4:10,104:10] \| k2[4:10,204:10]` | 通过 |
| 精度评估 | A/B 对 key=1 双星交会 | 1–80 | `dual_sat>0` | 80/80，交会 RMSE 169 m | 通过 |
| 对照 | 测角 RMSE / 逐周期误差抖动 vs 原场景 kEkf | 全程 | 见下表 | 角度 KF 测角 RMSE 约 2.1 倍、误差步长约 4 倍；变化率与几何角速度同量级 | 通过（对照成立；本几何下 EKF 更稳） |

### 对照摘要（同库、同几何、各跑 80 周期）

| 指标 | 原场景 kEkf | 本场景 kAngleCvKf |
| --- | ---: | ---: |
| 测角 RMSE | 7.67×10⁻⁵ °（1.34 μrad） | 1.60×10⁻⁴ °（2.80 μrad） |
| 测角 p95 | 1.36×10⁻⁴ ° | 2.67×10⁻⁴ ° |
| 误差步长 RMSE（az / el） | 3.8×10⁻⁵ / 4.2×10⁻⁵ ° | 1.55×10⁻⁴ / 1.54×10⁻⁴ ° |
| 双星交会 RMSE | 92 m | 169 m |
| 本星会话测角 RMSE（末周期） | 5.2×10⁻⁵ / 5.8×10⁻⁵ ° | 1.17×10⁻⁴ / 1.07×10⁻⁴ ° |
| 甲方 3 μrad 门 | 过 | 过（2.80 μrad） |

视线本体步进（真值运动）两边几乎一样：方位约 0.0006–0.0009°/周期，俯仰约 0.0024°/周期。角度 KF 估计的俯仰变化率 RMSE 0.00240°/s，与这条几何角速度同量级。

## 产物

- `examples/log/sbirs_geo_triple_angle_kf/angle_track.csv`：滤波后方位/俯仰航迹
- `examples/log/sbirs_geo_triple_angle_kf/pe_angular.csv`：精度评估角度误差样本
- `examples/log/sbirs_geo_triple_angle_kf/pe_dual_sat.csv`：双星交会位置误差
- `sbirs_acceptance.log`：`目标角度状态估计`（含变化率）

## 构建与运行

```bash
source scripts/activate_1q_git_bash.sh
scripts/1q.sh configure VisualStudio.15.0-amd64   # 新增场景目录后需要一次
scripts/1q.sh build VisualStudio.15.0-amd64-release --target sbirs_geo_triple_angle_kf
./build/VisualStudio.15.0-amd64/Release/bin/sbirs_geo_triple_angle_kf.exe
```

对照原场景：

```bash
scripts/1q.sh build VisualStudio.15.0-amd64-release --target sbirs_dual_sat_fix_messages
./build/VisualStudio.15.0-amd64/Release/bin/sbirs_dual_sat_fix_messages.exe
```

## 结论

**判定：通过。** 三 GEO + 消息地面站上 `kAngleCvKf` 已进入 NFOV 估计跟踪，滤波方位/俯仰、变化率、角度航迹均可观测。

**稳定性对照：本几何下默认 kEkf 更稳、更准。** 原因不是角度 KF 发散，而是生产 EKF 用场景真值三维起步，角度 KF 只用测角。两边都在 3 μrad 指标内；角度 KF 的变化率与 GEO 斜距上目标相对角速度一致。落点/发射点 RMSE 仍是中段平飞对弹道外推的几何错配，不代表交会失败。
