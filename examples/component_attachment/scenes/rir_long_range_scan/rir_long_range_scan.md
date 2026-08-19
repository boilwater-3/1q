# 场景预期表：rir_long_range_scan（远程识别雷达远距探测能力）

## 场景元数据

| 项 | 值 |
| --- | --- |
| 场景文件 | `scenes/rir_long_range_scan/rir_long_range_scan.json` |
| 场景意图 | 被测通道：RIR（主）；被测行为：甲方链路参数下，3400 km / RCS 0.025 m² 与 8550 km / RCS 1 m² 两目标在扫描体积内被探测并形成航迹；验证深度：L2 预期表 + L3 几何/链路先验 |
| 构建模式 | release（运动学回退） |
| 日志模式 | delta + key（默认） |
| 目标设定 | 两目标同方位正东、俯仰 20°（扫描体积内：方位 ±110°、俯仰 2°–90°），沿北向穿过扫描扇区 |
| 运行日期 | 2026-08-20 |

## 链路与配置说明

甲方给出的可填参数已写入 `examples/configs/remote_identification_radar.json`：

| 甲方量 | 填入字段 | 值 |
| --- | --- | --- |
| 阵面方位 220° | `mission.scan.scan_limits_deg.az_*` | −110° ~ +110°（雷达局部系 az 0 = 东，合法域不能跨过 ±180°） |
| 俯仰 2°–90° | `el_min_deg` / `el_max_deg` | 2 / 90 |
| 峰值功率 582 kW | `hardware.transmitter.peak_power_w` | 582000 W |
| 天线增益 52 dB | `hardware.antenna.main_beam_gain_db` | 52 |
| 脉宽 16 ms | `hardware.transmitter.pulse_width_s` | 0.016 s |
| 积累次数 1 | `policy.detection.pulse_count` | 1 |

甲方未给、但必须配套改的量（否则会话校验或回波时序会拒绝这些参数）：

- `maximum_peak_power_w` = 582000（峰值不能超过上限）
- `maximum_pulse_energy_j` = 10000（16 ms × 582 kW = 9312 J，原上限 20 J 会拒）
- `maximum_duty_cycle` = 0.5（16 ms × 原 PRF 300 Hz 占空比 > 1）
- `prf_hz` = 10（须 PRF < 17.5 Hz 才盖得住 8550 km 不模糊距离；且脉宽必须 < PRI/2）
- `mission.max_range_m` = 9e6（原 300 km 会把两个目标都裁掉）
- `recognition_dwell_sec` = 0.1 s（8550 km 双程时延 ≈ 57 ms，原 50 ms 接收窗收不到回波）

手算单脉冲处理后 SNR（脉压 B·τ、发射/接收损耗 3.5/2 dB、噪声系数 4 dB、S 波段 3 GHz、带宽 4.5 MHz 沿用示例缺省）：两目标约 **7.9 dB**，刚好过 6 dB SNR 回退门。两者 RCS 比 = 40、距离比⁴ ≈ 40，是同一条等 SNR 轮廓。

## 几何先验（L3）

俯仰固定 20°（扫描体积内，避开 2° 地平线边缘）：

| 目标 | 斜距 | 水平距离 `range_m` | 高度 `altitude_m` | 场景方位 | 雷达局部方位 |
| --- | --- | --- | --- | --- | --- |
| 1001 RCS 0.025 m² | 3400 km | 3194955 m | 1162868 m | 90°（东） | 0° |
| 1002 RCS 1 m² | 8550 km | 8034372 m | 2924272 m | 90°（东） | 0° |

北向速度按距离比例（2500 / 6287 m/s），400 s 末雷达方位约 +17°，仍在 ±110° 内；俯仰约 19°，仍在 2°–90° 内。

默认 `enable_directional_pattern=false`：探测按主瓣峰值增益，不要求光栅波束刚好压在目标上。扫描限位仍约束驻留中心，本场景不指定目标（`designated_target_id=0`），波束按扫描策略空转。

## 预期事件表

| 通道 | 行为 | 预期周期窗 | 预期量级 | 预期计数 | 实测 | 判定 |
| --- | --- | --- | --- | --- | --- | --- |
| RIR | 双目标进入航迹（探测成立） | cycle 1 起 | 视图 `航迹≥2`；斜距量级 3400/8550 km | 全程多数周期 | cycle 1 即 `航迹=2`；400/400 周期 `航迹≥2`（2/3/4/5 = 88/124/141/47）；ENU 与先验逐位相符 | 通过 |
| RIR | 目标保持在扫描体积内 | 全程 400 周期 | 雷达方位 0°→~17°，俯仰 ~20° | 400 | 末周期 ENU 推方位 17.3°、俯仰 19.2°；驻留中心方位扫过 −110°~+110°，俯仰 90°→62°（400 周期未扫完 2° 底） | 通过 |
| RIR | 两目标 SNR 同量级 | 探测周期 | ~8 dB（过 6 dB 回退门） | 与航迹同期 | 两目标同期探测成立（等 SNR 轮廓）；视图不落 SNR 数值，手算 7.9 dB | 通过 |
| RIR | 型号确认 | 不作为本场景门控 | 真值型号不在交付库 | — | cycle 32 起最近邻误配 MQ-1C / F-16C（置信 0.44/0.41）；非本场景被测项 | 按设计（非门控） |
| Fusion | RIR 量测并键 | cycle 1 起 | 源通道 5，峰值 ≥1 | ≥1 | cycle 1 `fused=2`，推理源类型=5 | 通过 |
| 其余通道 | 远距目标不在机载窗内 | — | EOS/AR 无探测属按设计；SAR 产品与点目标解耦 | SAR 产品 ≥1 | AR `snr_below_threshold`、EOS/SBIRS 视场外、SAR 产品 1（squint 拒绝穿插） | 通过 |

## 冒烟下限

min_key_events=1 / min_sbirs_events=0 / min_sar_products=1 / min_fused_targets=1 /
min_rir_recognition_outputs=0（识别确认不是本场景被测行为）

实测：events=61 / sbirs=0 / sar_products=1 / fused=2 / rir_views=400 /
rir_confirmed_cycles=276 / **SMOKE exit=0**。

## 结论

**判定：通过**（探测能力成立）。日志：`/tmp/1q/scenes/rir_long_range_scan/`。

两点附带观察（不否定“能探测”）：

1. **远距高速航迹分裂**：目标北向 2.5 / 6.3 km/s，内部 KF 量测噪声仍是 10 m 量级，
   后期 `航迹` 升到 3–5，且 1002 在视图里出现重复条目——这是跟踪关联在远距/高速下的
   既有口径，不是链路预算失败。
2. **识别最近邻误配**：交付库没有 `RCS-0.025` / `RCS-1.0` 型号，匹配器仍输出
   MQ-1C / F-16C。本场景不把型号确认当验收项。
