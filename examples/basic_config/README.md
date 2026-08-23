# 会话配置 JSON 文件说明

本目录包含 AR、EOS、ESR、SAR、SBIRS、RIR 六个模块的完整会话配置 JSON 文件，供示例程序加载使用。

## 文件清单

| 文件 | 对应模块 | 说明 |
| --- | --- | --- |
| `airborne_radar.json` | 机载雷达 | 完整 `ArSessionConfig`，约 85 个叶子字段 |
| `electro_optical.json` | 光电传感器 | 完整 `EosSessionConfig`，约 30 个叶子字段 |
| `electronic_warfare.json` | 电子侦察 | 完整 `EsrSessionConfig`，约 40 个叶子字段 |
| `sar.json` | 合成孔径雷达 | 完整 `SarSessionConfig`，使用与 SAR integration demo 同类的自洽参数 |
| `sbirs.json` | 天基红外传感器 | 完整 `SbirsSessionConfig`，供 component_attachment demo 加载 |
| `remote_identification_radar.json` | 远程识别雷达 | 完整 `RirSessionConfig`，供 component_attachment demo 加载（识别库路径相对 `examples/basic_config/`，运行时由 `CA_RIR_DATABASE_PATH` 解析） |

## remote_identification_radar/ 子目录

`remote_identification_radar/` 存放远程识别雷达（RIR）目标识别数据库的两类资产
（**非示例杂物，被工具与集成测试真实消费**）：

| 文件 | 角色 |
| --- | --- |
| `recognition_database_input.json` | 建库输入源——`tools/remote_identification_radar_db_builder.py --input` 的唯一输入 |
| `target_feature_database_v1.1.db` | 交付库（SQLite）——由建库工具产出，被 RIR 识别集成测试经 `ONEQ_RIR_EXAMPLE_DATABASE_PATH` 加载（`tests/integration/remote_identification_radar/`） |

## JSON 结构与配置说明

每个 JSON 文件的顶层键与对应模块的 `*SessionConfig` 结构体字段一一映射。

### 公共配置域结构

基线四域（所有有会话的传感器）：

```text
{
  "hardware":    { ... },   // 装备或传感器固有能力
  "mission":     { ... },   // 任务态与工作模式
  "policy":      { ... },   // 算法策略
  "environment": { ... },   // 场景环境语义输入
}
```

有静态安装指向几何的模块（SBIRS / AR / ESR）另含顶层 `"orientation"`（初始化静态；
不进 RuntimeConfigPatch）。EOS / RIR / SAR 保持四域，禁止空壳 orientation。

详细字段说明见 `docs/common/usage.md`（1q 库消费指南）与 `docs/common/session_contract.md`（会话相关模块契约）。
所有权规则见 `docs/common/contract.md`「条件五域配置所有权」。

### 物理链路提示

AR 始终使用物理探测链。`airborne_radar.json` 启用
`rcs_physics.enable_physical_rcs`，并将 `physics_mix_ratio` 设为 `1.0`，使示例直接体现
距离衰减与目标 RCS 的物理趋势。

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

示例程序通过轻量 JSON 解析器 `oneq::JsonReader` 加载配置文件，再通过域映射器
（`examples/common/config_loaders/<域>/config_loader.h`）将 JSON 树转换为对应的
`*SessionConfig` 结构体。

相关文档：
- `docs/common/usage.md` — 1q 库消费指南
- `docs/common/session_contract.md` — 会话相关模块契约（SessionConfig 直接赋值、RuntimeConfigPatch 显式 has_*、运行期配置提交策略、三层输出模型等）
