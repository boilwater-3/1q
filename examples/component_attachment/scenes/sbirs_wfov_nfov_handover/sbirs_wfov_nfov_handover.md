# 场景预期表：sbirs_wfov_nfov_handover（宽窄视场交接 + 验收量）

## 场景元数据

| 项 | 值 |
| --- | --- |
| 场景文件 | `scenes/sbirs_wfov_nfov_handover/sbirs_wfov_nfov_handover.json` |
| 场景意图 | 被测通道：SBIRS（主）；被测行为：WFOV 扫描发现 → 连续命中门 → NFOV 指向捕获 → 持续跟踪的完整宽窄交接链，以及 `[SbirsAccept]` 验收事件流的消费示范（需求 3.2.1.3 验收量） |
| 构建模式 | release（运动学回退；SBIRS 几何与平台动力学解耦） |
| 日志模式 | delta + key（默认） |
| 单变量 | 相对 `sbirs_altitude_snr_1000km`：星高 1000→500 km（SNR 回到捕获跟踪区间）+ 显式焦平面参数（2.0 m / 30 μm）+ `wide_to_narrow_required_consecutive_hits` 1→2 |
| 运行日期 | 2026-08-19 |

## 链路先验

SNR ∝ 1/R²（同 1000 km 专项场景标定）：500 km 基线实测 SNR 20.0/21.7 linear，
远过 wide（4.0）与 narrow（6.0）双门限 → 发现 + 捕获 + 跟踪成立。连续命中门 = 2：
候选须连续两个周期有效命中才允许进入 NFOV 调度（单通道 max_locks=1，两目标
竞争 → 一目标锁定、另一目标调度跳过）。

## 验收日志开闸方法（本场景核心示范点）

`[SbirsAccept]` 事件流是库编译期开关，默认关闭（关闭时宏与派生计算一并剪除、
零开销）。开启方法：

```bash
cmake --preset llvm-ninja-release-local -DENABLE_EXAMPLES=ON \
      -DONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG=ON
cmake --build --preset llvm-ninja-release-local --target component_attachment_demo
./build/llvm-ninja-release-local/bin/component_attachment_demo \
    --scene examples/component_attachment/scenes/sbirs_wfov_nfov_handover/sbirs_wfov_nfov_handover.json
grep "SbirsAccept" examples/component_attachment/log/1q_library.log | head
```

七类事件（全在本场景出现）：

| 事件 | 携带的验收量 | 本场景实测计数 |
| --- | --- | --- |
| `scan_footprint` | 地面覆盖区四角/中心经纬度（指向太空记 miss）、驻留时间、扫描率 | 400 |
| `wfov_candidate` | 真值/量测角、az/el 误差、SNR、d_max、接收功率、信号能量、连续命中计数 | 45 |
| `wfov_hit_gate` | 连续命中门挡下记录（hits/required） | 23 |
| `nfov_acquisition` | 指向/捕获判定（slewing/timeout/rejected/captured/failed） | 22 |
| `nfov_track` | 焦平面脱靶量（米 + 像素，`focal_valid`）、指向误差、SNR/NIS、门失败计数 | 44 |
| `nfov_schedule` | 选中列表、通道分配、locked/max、resources_full、skipped | 400 |
| `nfov_release` | 通道释放与原因、失败计数 | 22 |

## 预期事件表

| 通道 | 行为 | 预期周期窗 | 预期量级 | 预期计数 | 实测 | 判定 |
| --- | --- | --- | --- | --- | --- | --- |
| SBIRS | WFOV 首次命中被门挡 | cycle 4 | `consecutive_hits=1 required=2` | ≥1 | cycle 4（1001） | 通过 |
| SBIRS | 连续 2 命中放行 + 捕获 | cycle 5 起 | `outcome=captured`，hits=2/2 | ≥1 | cycle 5（1001）/22（1002） | 通过 |
| SBIRS | NFOV 持续跟踪 | 捕获后 | `focal_valid=true`，脱靶量米+像素 | ≥10 | 44 | 通过 |
| SBIRS | 调度竞争 | 全程 | 单通道：一锁定一 skipped | 每 36 周期窗口 | 400 条 schedule | 通过 |
| Fusion | 融合目标 | cycle 1 起 | 峰值 ≥ 1 | ≥1 | 正常 | 通过 |
| SAR/AR/ESR/EOS | 与基线一致 | — | — | ≥ 基线部分量 | 正常 | 通过 |

## 冒烟下限

min_key_events=1 / min_sbirs_events=10 / min_sar_products=1 / min_fused_targets=1
（实测 sbirs_events=67，exit=0 通过）

## 结论

**判定：通过**。连续命中门行为符合设计：1001 于 cycle 4 首次命中（1/2 被挡、
`wfov_hit_gate` 落记录）、cycle 5 第二次命中（2/2）进入调度并同周期捕获；1002
同形态（cycle 22/23）。捕获后 NFOV 通道锁定跟踪，焦平面脱靶量按 x=f·tanΔaz
映射输出米 + 像素双口径。验收量只经日志通道、不进公开输出结构（模块非目标 10
口径不变）；本场景焦平面/命中门字段经 `sbirs_satellite` 场景块覆写，装载路径
sbirs.json → LoadSbirsHardware/LoadSbirsScheduler → ApplySceneOverrides。
