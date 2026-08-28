# examples/ — 场景示例集（每场景一个可执行）

示例层回答"消费方怎么把 1Q 库组装成一台仿真机"：core ECS 骨架 + components
组件封装 + app 装配运行体 + scenes 场景集。**主开发路径是每场景独立可执行**
（`scenes/<name>/main.cpp` 薄入口，场景 JSON 由编译宏钉死）；对外兼容目标
`component_attachment_demo`（通用 runner，`--scene` 跑任意场景）与
`precision_evaluation_demo`（= 精度评估场景 `sbirs_dual_sat_fix` 的同源目标）保持原名。

## 快速开始

```bash
cmake --preset examples-release-local            # 一次性 configure（含示例）
cmake --build --preset examples-release-local --target rir_jammed_scan   # 构建某场景
./build/examples-release-local/bin/rir_jammed_scan                        # 运行（日志落 examples/log/rir_jammed_scan/）
./build/examples-release-local/bin/rir_jammed_scan --cycles 120          # 缩短迭代
# 想跑任意场景 JSON：通用 runner
./build/examples-release-local/bin/component_attachment_demo --scene examples/scenes/<name>/<name>.json
# 可视化
python3 examples/common/viz/build_viewer.py examples/log/<name>          # 通用单文件 HTML 查看器
# RIR 扫描-识别专用查看器（LLA 俯视 + 扫描扇区 + 进入扫描范围→探测→确认航迹→识别连线 + 斜距/高度剖面）
python3 examples/common/viz/rir_scan_viewer.py examples/log/rir_ground_site_recognition
```

场景可执行选建（configure 期）：`-DONEQ_EXAMPLE_SCENES=all|none|"名1;名2"`
（默认 all；未知场景名 configure 报错）。新建场景 = `scenes/` 下建目录放
`<name>.json + <name>.md + main.cpp` 三件套即被 CMake 自动收录（拷任一现有
main.cpp 改文件头注释即可）。

## 目录结构

```
examples/
├── core/            ECS 内核（组件基类/实体/世界/Boost.Signals2 信号）+ 场景共享状态
├── components/      每个仿真模块一个组件（飞行/AR/ESR/ECM/EOS/SBIRS/SAR/RIR/融合/推演/威胁）
├── logger/          集成端日志（双后端：spdlog / Windows ofstream；验收路径钉扎）
├── app/             装配层：RunScene 运行体 + 可视化 CSV 落盘 + 指令路由 + 通用 runner main
├── scenes/          场景集：每场景目录（main.cpp + JSON + MD 期望表）+ 场景加载代码
│                    （scene_data/scene_script/area_division）；schema 权威见 scenes/README.md
├── basic_config/    六域基础 session 配置模板 + RIR 识别库资产（新场景 session_config 的拷贝源）
├── common/          examples 与 tests 共享便利层（JSON 解析/CSV/配置加载器/可视化查看器）
└── log/             场景运行产物（git 忽略；每场景子目录由场景 JSON 的 log_dir 声明）
```

## 场景自持配置（schema v2）

每个场景 JSON 自带三样东西：**场景真值**（平台/目标/指令/冒烟）、
**session_config**（挂载即全量：挂载通道必带从 `basic_config/` 模板整份拷贝的
会话配置，未挂载禁止携带）、**log_dir**（本场景日志目录，相对 `examples/log/`；
运行期硬拒临时目录——`--output-dir` 覆盖同样受约束）。详见
[`scenes/README.md`](scenes/README.md) 的「场景描述文件」节。

## 配置注入约定（编译宏，CMake 注入）

| 宏 | 消费方 | 说明 |
| --- | --- | --- |
| `SCENE_CONFIG_DIR` | examples_core 与单元测试 | 指向 `examples/basic_config/`（session_config 模板与 config_loader 夹具源） |
| `CA_RIR_DATABASE_PATH` | examples_core 与单元测试 | RIR 识别库绝对路径（Windows 交付免相对路径歧义） |
| `CA_SCENE_DIR` | component_attachment_demo | 指向 `examples/scenes/`，`--scene` 默认值 |
| `CA_DEFAULT_OUTPUT_DIR` | examples_core | 日志根 `examples/log/`（+ 场景 `log_dir` = 默认输出目录） |
| `ONEQ_SCENE_JSON` | 每场景可执行 | 本场景 JSON 绝对路径（薄入口钉死） |
| `PE_DEFAULT_OUTPUT_DIR` / `PE_ACCEPTANCE_LOG_ENABLED` | 精度评估场景 | 同一日志根 / 验收日志开关透传 |
| `CA_VIEW_LOG_MODE` / `CA_EVENT_LOG_MODE` | examples_core | 集成端日志模式（summary/nonnominal/delta + all/key/aggregate，见 `logger/logger_modes.h`） |

## 相关文档

- [`scenes/README.md`](scenes/README.md) — 场景系统权威：JSON schema v2、场景集、六自由度/巡逻/多机/天基设计
- [`core/README.md`](core/README.md) — ECS 核心设计（组件/实体/世界/信号、挂载序、运行期修改接口）
- [`logger/README.md`](logger/README.md) — 集成端日志设施（三模式宏门控）
- [`basic_config/README.md`](basic_config/README.md) — 六域基础配置模板与加载方式
- `docs/practice/output_view_and_logging_guide.md` — 输出查看与日志全景
- `docs/practice/customer_integration_verification.md` — 交付验收操作手册
