# 文档可靠性审查闭环报告

Status: draft

**Review-Date:** 2026-07-20

**Closure-Date:** 2026-07-21

**Authority:** 非规范性审查记录；不得替代 `docs/common/contract.md`、各模块 `design.md` 或 `docs/practice/`。

## 1. 范围与判定方法

本轮以 live code、tests、schema、codec、runtime resolver、构建配置和模块权威设计为证据，复核文档中的事实描述。`flight_dynamic` 不作为被审查模块；它只在跨模块依赖或构建边界需要说明时出现。

结论按以下类别归档：

1. **过时事实**：描述曾经成立，但已落后于当前实现；
2. **文档错误**：描述与当前可验证契约不符；
3. **真实代码缺陷**：实现违反已经固化的设计或契约；
4. **设计/实现分歧**：两者不同，但需要先裁决目标语义；
5. **继续 defer**：当前没有足够证据或不属于本批范围。

已裁决规则均迁入对应权威文档；未裁决边界只进入 `open_questions.md`。本报告不保存第二份设计方案。

## 2. 总体状态

| 范围 | 状态 | 权威落点 |
|---|---|---|
| AR | 已关闭 | `docs/airborne_radar/design.md` |
| ESR | 已关闭；剩余非阻塞问题继续开放 | `docs/electronic_surveillance_radar/design.md`、`docs/common/open_questions.md` |
| SAR | 已关闭 | `docs/sar/design.md` |
| EOS | 已关闭 | `docs/electro_optical_sensor/design.md` |
| SBIRS | 已关闭；剩余非阻塞仿真边界继续开放 | `docs/space_based_infrared_sensor/design.md`、`docs/common/open_questions.md` |
| Common | 本批已关闭；Windows/MSVC 全链证明继续开放 | `docs/common/contract.md`、`docs/common/usage.md`、`docs/common/open_questions.md` |
| Practice | 本批已关闭 | `docs/practice/batch_validation.md`、`docs/practice/ci.md` |

## 3. Common 证据矩阵

| 编号 | 原问题 | 证据判定 | 分类 | 处置 |
|---|---|---|---|---|
| COMMON-01 | “禁止异常”被写成全工具链 `-fno-exceptions` 保证 | 构建没有提供该全局保证；项目代码不应以抛异常表达失败，但现有第三方边界可捕获并转换成项目状态 | 文档错误 | 在 contract 固化为“不新增 throw；窄边界 catch 后转换”，删除虚假的编译器保证 |
| COMMON-02 | `src/common` 被描述为只能使用 `oneq::common` | 共享实现既可实现公开域命名空间，也可承载 `oneq::common::<domain>` 内部设施 | 文档错误 | 按所有权和公开边界定义命名空间，不由目录名机械决定 |
| COMMON-03 | Session 所有权只覆盖四个传感器模块 | live API 已包含 AR、EOS、ESR、SAR、SBIRS 五个模块，SBIRS 同样具有 Session/runtime patch 所有权 | 过时事实 | 更新为五模块契约 |
| COMMON-04 | ESR 输出被概括成单一探测结果 | live 输出包含三类业务结果和 `sensor_powered_off` 执行元数据 | 过时事实 | contract 改为结构化结果与执行元数据 |
| COMMON-05 | Windows preset 被当作 Windows 支持证明 | preset 和批处理脚本只证明存在脚手架，尚无真实 Windows 的依赖获取、配置、构建、安装和消费者闭环 | 设计/实现分歧 | 保留 shell/GitHub 依赖引导目标；证明门槛进入 COMMON-OQ-1 |
| COMMON-06 | Common 文档清单漏掉 usage | 当前稳定文档为 contract、open questions、usage | 过时事实 | 文档治理清单和结构守卫同步更新 |
| COMMON-07 | 数据流图暗示 flight_dynamic 直接驱动所有传感器 | 仓库内没有该直接依赖；CycleInput 由外部编排器构造，flight_dynamic 只是可选数据来源 | 文档错误 | 图示改为外部编排边界 |
| USAGE-01 | 安装包被称为“零依赖” | 安装后的目标仍有公开头文件、标准库和按功能出现的第三方消费边界 | 文档错误 | 改为 1q 安装产物与 consumer-provided dependencies 的明确边界 |
| USAGE-02 | 示例声明可直接使用 `requires = "1q/0.1"` | 当前 `conanfile.py` 负责依赖和工程构建环境，不提供完整 1Q Conan package 生命周期 | 文档错误 | 删除虚构的 Conan 消费契约，保留 CMake install/consumer 路径 |
| USAGE-03 | 库类型和宏被写成固定值 | 目标类型跟随 `BUILD_SHARED_LIBS`；`ONEQ_STATIC_DEFINE` 仅用于静态消费 | 过时事实 | usage 按 live target 属性说明 |
| USAGE-04 | C++ 标准和生成配置文件名被写死 | 默认标准为 C++17；生成文件和路径应由 CMake package contract 消费，不应由文档猜测 | 文档错误 | 只承诺已验证的 CMake target 用法 |
| USAGE-05 | 安装逻辑声称会生成可移植第三方快照 | 查找路径硬编码目录与 `x86_64`，当前 Apple Silicon 构建实际找不到 Conan data；即使生成，header-only stand-in 也可能遮蔽真实二进制依赖 | 真实代码缺陷 | 删除不完整的第三方快照机制；安装树只导出 1q，自身依赖由 consumer 环境完整解析 |

## 4. Practice 证据矩阵

| 编号 | 原问题 | 证据判定 | 分类 | 处置 |
|---|---|---|---|---|
| BATCH-01 | 场景总数和模块分布为历史快照 | live `--list-scenarios` 为 199 个 sweep、31 个 sequence，共 230 个场景 | 过时事实 | 更新当前快照，并指定 `--list-scenarios` 为数量事实源 |
| BATCH-02 | replay divergence 被描述为 warning | batch runner 将 trace/replay 失败或分叉视为阻塞错误并返回非零；只有物理趋势属于 warning | 文档错误 | 固化阻塞/告警边界 |
| BATCH-03 | EOS replay 路径及通用 matrix 测试模式已失真 | EOS replay 测试位于专属 replay 目录；五模块不存在统一的 `tests/unit/*_matrix_test.cpp` 契约 | 过时事实 | 文档改为 live target、CTest label 和源码清单 |
| CI-01 | Windows preset 存在即等于 Windows 支持 | 当前 CI 只在 macOS 运行；Windows 链尚未经真实 runner 证明 | 过时事实 | CI 文档明确“脚手架，不是支持声明”并引用 COMMON-OQ-1 |
| CI-02 | contract 测试数量被硬编码 | 测试注册会随守卫和编译目标变化；数量不是稳定契约 | 过时事实 | CI 与文档只依赖 `contract` label，不固定计数 |

## 5. 已冻结结论

- 项目失败语义不得依赖 C++ 异常；不新增 `throw`。已有第三方边界只允许窄范围捕获并转换为项目状态。
- `src/common` 的命名空间由公开域和所有权决定，不由目录名决定。
- Common 契约覆盖五个传感器模块；跨模块周期输入由外部编排器拥有。
- 安装文档不承诺零依赖，也不宣称当前仓库提供可 `requires` 的 1Q Conan 包。
- 安装树不制造第三方包的局部替身；完整依赖 target 由 consumer 环境提供。
- Windows preset/脚本目前是未验证脚手架。支持声明必须以真实 Windows 全链通过为准。
- batch validation 中，结构化检查、trace/replay 失败和分叉是阻塞错误；物理趋势才是非阻塞告警。
- 场景数量与 contract 测试清单属于 live inventory，不作为长期硬编码契约。

## 6. 继续开放的问题

唯一规范入口为 [`docs/common/open_questions.md`](../common/open_questions.md)：

- COMMON-OQ-1：Windows/MSVC shell/GitHub 依赖引导及真实 runner 全链证明；
- SBIRS-OQ-1～OQ-4：SBIRS 非阻塞仿真边界。

2026-07-24 follow-up：原 ESR-OQ-1 的 runtime patch 全域校验已进入 ESR design 与 common contract，
不再属于开放问题。

## 7. 本批验证记录

- live batch inventory：199 个 sweep + 31 个 sequence = 230；
- 五个 `batch_validation::<module>` CTest 分区通过；
- `contract::public_api`、文档结构、安装清单、public API 边界、legacy term、测试布局和 preset provider 守卫通过；
- release preset 重新配置并构建相关公共 API 与 batch validation 目标；
- release install 成功，独立 consumer 的 9 个可执行目标完成配置、编译和链接；
- 完整 `contract` label 共 24 项通过。
