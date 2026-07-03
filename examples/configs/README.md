# 会话配置 JSON 文件说明

本目录包含 AR、EOS、ESR、SAR 四个模块的完整会话配置 JSON 文件，供示例程序加载使用。

## 文件清单

| 文件 | 对应模块 | 说明 |
| --- | --- | --- |
| `airborne_radar.json` | 机载雷达 | 完整 `ArSessionConfig`，约 85 个叶子字段 |
| `electro_optical.json` | 光电传感器 | 完整 `EosSessionConfig`，约 30 个叶子字段 |
| `electronic_warfare.json` | 电子侦察 | 完整 `EsrSessionConfig`，约 40 个叶子字段 |
| `sar.json` | 合成孔径雷达 | 完整 `SarSessionConfig`，使用与 SAR integration demo 同类的自洽参数 |

## JSON 结构与配置说明

每个 JSON 文件的顶层键与对应模块的 `*SessionConfig` 结构体字段一一映射。

### 公共四域结构

```text
{
  "hardware":    { ... },   // 装备或传感器固有能力
  "mission":     { ... },   // 任务态与工作模式
  "policy":      { ... },   // 算法策略
  "environment": { ... },   // 场景环境语义输入
}
```

详细字段说明见 `docs/public_model_config_manual.md`。

### 物理链路开关提示

`airborne_radar.json` 默认启用 `hardware.enable_physics_detection` 与
`rcs_physics.enable_physical_rcs`，并将 `physics_mix_ratio` 设为 `1.0`，使示例直接体现
距离衰减与目标 RCS 的物理趋势。若消费方需要完全确定性的简化检出路径，可在本地配置中
显式关闭这两个开关。

`sar.json` 的 `sample_rate_hz`、`pulse_width_s` 与 `range_sample_count` 必须满足
`ceil(pulse_width_s * sample_rate_hz) <= range_sample_count`。当前示例配置已按
`1MHz * 20us <= 1024` 保持自洽。

### 枚举值表示

所有 C++ 枚举在 JSON 中使用字符串表示（如 `"kEsm"`、`"kBalanced"`、`"kGaussianMainLobe"`）。

### 数值类型

- 浮点字段使用 JSON number
- 整数字段使用 JSON number
- 布尔字段使用 JSON `true` / `false`

### 数组与向量

`std::vector` 类型的字段使用 JSON 数组。当前配置文件中暂无填入元素的示例；空数组表示默认空列表。

## 加载方式

示例程序通过轻量 JSON 解析器 `oneq::JsonReader` 加载配置文件，再通过域映射器（`examples/*_config_loader.h`）将 JSON 树转换为对应的 `*SessionConfig` 结构体。

相关文档：
- `docs/public_target_input_manual.md` — 单周期目标输入说明
- `docs/public_module_output_manual.md` — 单周期输出结构说明
