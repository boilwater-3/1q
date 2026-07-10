# 测试编写架构重规划

Status: draft
Authority: test architecture replan proposal
Date: 2026-07-10
Live-Baseline: `b9ba6a0f`
Implementation-State: Phase 0-1 complete; Phase 2-6 pending

## 阶段进度

- **Phase 0：冻结基线和自动守护** — ✅ 完成（2026-07-11）
- **Phase 1：建立类型 × 域目录** — ✅ 完成（2026-07-11）
- Phase 2：拆分 unit 分区目标 — ⏳ pending
- Phase 3：拆分 replay/integration/contract/performance/compatibility — ⏳ pending
- Phase 4：消除专项 filter 和迁移 CI 策略 — ⏳ pending
- Phase 5：覆盖率、工具和活跃文档收口 — ⏳ pending
- Phase 6：全矩阵验收和旧结构删除 — ⏳ pending

## 结论

当前 `tests/CMakeLists.txt` 的机械拆分已经解决了单文件过长，但没有解决测试作者面对的核心问题：测试类型、业务归属、构建目标、CTest 标签和 CI 门禁仍是五套没有统一规则的分类方式。`SarTests.cmake` 之所以显得特殊，是因为 SAR 的 unit、replay、integration、contract、performance、compatibility 被一组手写 GoogleTest filter 临时拼成了专项矩阵；这不是应该推广的长期模型。

推荐将测试架构冻结为两个正交维度：

1. **测试类型**决定测试放在哪个一级目录：`unit`、`integration`、`replay`、`contract`、`performance`、`compatibility`、`consumer`。
2. **业务域**决定测试放在哪个二级目录、编译进哪个分区目标：`common`、`airborne_radar`、`electronic_surveillance_radar`、`electro_optical_sensor`、`sbirs_sensor`、`sar`、`flight_dynamic`、`cross_domain`、`project`。

构建时，每个有效的“类型 × 域”分区形成一个 GoogleTest 可执行文件和一个 CTest 项。CTest label 同时携带类型、业务域和执行策略，CI 不再依赖 `sar_ci` 之类模块特例。现有 `1q_unit_tests` 等构建目标名在迁移完成后保留为 aggregate build target；固定二进制路径不作为兼容合同，直接运行它们的脚本需要同步迁移。

## 规划边界

本方案从测试编写和维护体验出发，不以“把 CMake 再拆成更多文件”为目标。

本轮规划包含：

- 测试类型的判定规则；
- 测试目录、目标、CTest 名称和 label 合同；
- SAR/FD 专项 filter 的消除路径；
- 现有脚本、CI、覆盖率和历史命令的兼容迁移；
- 分阶段实施、验证和回滚边界。

本轮规划不包含：

- 修改产品代码或测试断言；
- 为了分目录而批量重命名 GoogleTest suite/case；
- 改变 known-limit、性能阈值或 CI 阻断级别；
- 首批引入 `gtest_discover_tests()` 并把约 1476 个 case 全部展开成 CTest 项；
- 把安装后 consumer 验证合并进普通进程内 GoogleTest。

## 当前事实基线

### 源文件规模

| 当前目录 | 文件数 | 主要问题 |
|---|---:|---|
| `tests/unit` | 178 | 所有业务域平铺；replay、session/pipeline 集成测试和 example helper 测试混入 unit |
| `tests/integration` | 5 | AR/EOS/ESR/SBIRS/cross-domain 各一项，没有 SAR 独立源文件 |
| `tests/contract` | 8 个 C++ + 19 个 CMake guard | public API、源码边界、文档治理、C++11 compatibility 混在同一层 |
| `tests/performance` | 1 | 当前仅 SAR FFT 性能测试 |
| `tests/consumer` | 9 个 consumer 源文件 | 安装后外部消费验证，语义独立且合理 |

按当前 unit 文件名前缀粗分：SAR 66、AR 41、EOS 19、ESR 18、SBIRS 17、FD 8，另有 common/coordinate/estimation/numeric/json/replay 等 9 个文件。当前 README 只规定 `airborne/esr/eos` 前缀，已经不能描述现有模块全集。

### 当前构建和注册行为

- `TestSupport.cmake` 用递归 glob 收集四个类型目录，再由 `TestTargets.cmake` 创建 `1q_unit_tests`、`1q_integration_tests`、`1q_contract_tests`、`1q_replay_fast_tests`、`1q_performance_tests` 和可选 `1q_fd_tests`。
- `add_1q_gtest()` 的 `discovery_timeout` 参数没有被使用；当前并未调用 GoogleTest discovery。
- replay-fast 有 11 个源文件，其中 5 个 codec roundtrip 从 unit 目标移除，另外 6 个仍同时编译进 unit 和 replay-fast，存在重复编译和测试类型重叠。
- SAR integration 不是独立 integration 源集合，而是从 unit/replay 可执行文件中筛选 `SarSessionPipelineTest` 和 `SarReplaySessionTest`；同一用例因此同时承担多种类型语义。
- FD 五个 tier 和 SAR 多类型入口都依赖手写 suite/case filter。新增或重命名测试时，作者必须记得同步修改远端 CMake 清单，否则 CTest/CI 不会自动覆盖新用例。

### 已存在的兼容面

| 兼容面 | 当前依赖 | 迁移要求 |
|---|---|---|
| CMake build target | `1q_unit_tests`、`1q_contract_tests` 等 | 保留同名 aggregate build target |
| 固定二进制路径 | `tools/coverage_report.sh`、`tools/generate_maneuver_plots.sh` 直接运行 `bin/1q_unit_tests` | 同批改为分区发现或明确的新目标 |
| CI labels | `contract`、`sar_ci`、`integration`、`replay_fast`、`fd_ci` | 先等价映射，再迁到通用执行策略 label |
| 活跃文档 | `cmake/README.md`、`docs/practice/ci.md`、`docs/practice/coverage.md` | 与新 target/label 同批更新 |
| 历史 review | 多处记录旧二进制和 filter 命令 | 作为历史证据保留，不机械改写；必要时补“命令基于当时布局”说明 |
| 覆盖率 | 依赖单个 `1q_unit_tests` 作为 llvm-cov 主映射二进制 | 删除旧可执行文件前必须先验证新的 coverage runner/映射方案 |

## 测试作者的分类规则

新增测试时先判断类型，再判断 owner。文件名只描述行为，不再承担全局分类职责。

| 类型 | 判定问题 | 典型内容 | 不应放入 |
|---|---|---|---|
| `unit` | 是否验证一个组件/算法的局部行为，且不要求完整 session 流程？ | resolver、filter、geometry、validation、builder | 完整 session、trace roundtrip、安装消费 |
| `integration` | 是否跨越两个以上产品组件或完整执行一次 session/pipeline 场景？ | session、controller+pipeline、cross-domain scenario | 单个 codec、公共头编译 |
| `replay` | 核心断言是否是 trace/codec/snapshot/replay 的确定性、roundtrip 或 divergence？ | codec roundtrip、ReplaySession、trace compression/writer | 普通业务单元测试 |
| `contract` | 是否证明 public API、头文件、源码依赖方向、文档或构建结构的稳定边界？ | public convenience、header smoke、include direction、layout guard | 编译器标准兼容探针 |
| `performance` | 是否以耗时、吞吐、内存或规模阈值作为主要验收？ | FFT、批量处理基准 | 普通功能正确性 |
| `compatibility` | 是否证明某语言标准、编译器、profile 或最小消费环境可用？ | public header C++11、SAR C++11 source probe | public API 行为断言 |
| `consumer` | 是否必须从安装产物通过 `find_package(1q)` 构建外部程序？ | 九个安装 consumer | 仓库内直接链接 `${PROJECT_ALIAS}` 的测试 |

补充规则：

- `known_limit`、`smoke`、`fast`、`ci_required` 是执行策略，不是测试类型，使用 label 表达。
- 一个 `_test.cpp` 只能归属一个编译分区；共享 helper 必须放入 `tests/support/fixtures` 或 `tests/support/mocks`，文件名不得以 `_test.cpp` 结尾。
- 跨域场景归 `integration/cross_domain`，全项目 CMake/docs/public-boundary guard 归 `contract/project`。
- replay 测试不得为了进入 fast gate 再复制到 unit；它只属于 replay 分区，再附加 `fast` label。

## 目标目录

```text
tests/
├── CMakeLists.txt
├── README.md
├── cmake/
│   ├── TestSupport.cmake
│   ├── TestRegistry.cmake
│   ├── ScriptGuards.cmake
│   └── partitions/
│       ├── Unit.cmake
│       ├── Integration.cmake
│       ├── Replay.cmake
│       ├── Contract.cmake
│       ├── Performance.cmake
│       └── Compatibility.cmake
├── support/
│   ├── fixtures/
│   └── mocks/
├── unit/
│   ├── common/
│   ├── examples/
│   ├── airborne_radar/
│   ├── electronic_surveillance_radar/
│   ├── electro_optical_sensor/
│   ├── sbirs_sensor/
│   ├── sar/
│   └── flight_dynamic/
├── integration/
│   ├── airborne_radar/
│   ├── electronic_surveillance_radar/
│   ├── electro_optical_sensor/
│   ├── sbirs_sensor/
│   ├── sar/
│   ├── flight_dynamic/
│   └── cross_domain/
├── replay/
│   ├── common/
│   ├── airborne_radar/
│   ├── electronic_surveillance_radar/
│   ├── electro_optical_sensor/
│   ├── sbirs_sensor/
│   └── sar/
├── contract/
│   ├── project/
│   ├── public_api/
│   ├── cross_domain/
│   ├── airborne_radar/
│   ├── electronic_surveillance_radar/
│   ├── electro_optical_sensor/
│   ├── sbirs_sensor/
│   └── sar/
├── performance/
│   └── sar/
├── compatibility/
│   ├── public_api/
│   └── sar/
└── consumer/
```

目录只创建实际有测试的分区，不提交空目录。首轮移动保留现有文件名，避免“目录迁移 + 文件重命名 + fixture 重命名”同时发生；稳定后再单独评估是否去掉 `ar_`、`sar_` 等冗余前缀。

## 目标 CMake 模型

### 分区注册 API

测试 CMake 只保留一个通用注册入口，模块不再拥有专用注册机制：

```cmake
oneq_add_test_partition(
    TYPE unit
    DOMAIN airborne_radar
    SOURCES ${_sources}
    TIMEOUT 60
    LABELS ci_advisory
)
```

该函数负责：

1. 生成 target `1q_airborne_radar_unit_tests`；
2. 生成 CTest 项 `unit::airborne_radar`；
3. 设置 labels `unit;airborne_radar;ci_advisory`；
4. 统一链接 GTest、Boost、Eigen、FlatBuffers、ZLIB 和 `1q::core`；
5. 把所有 `_test.cpp` 记录到全局 registry；
6. 在 finalize 阶段证明每个测试源恰好注册一次，没有 orphan 或重复编译。

FD 所需 JSBSim include、link 和 data root 由 `DOMAIN flight_dynamic` 的 target requirement 处理；它是通用分区的依赖特例，不再拥有独立的测试注册架构。

### 源文件发现策略

测试目录允许使用局部 `file(GLOB ... CONFIGURE_DEPENDS)`，但只 glob 一个明确分区目录下的 `*_test.cpp`，不再从 `tests/unit` 根部递归收集整个仓库。选择这一策略是为了让测试作者新增文件后无需再编辑中心 source list。

为抵消 glob 的隐式性，必须同时具备：

- configure 时 registry 完整性校验；
- `test_layout_guard` 检查所有 `_test.cpp` 位于允许的“类型/域”路径；
- 新增/删除源文件后的 fresh configure 验证；
- CI 中输出最终分区和源文件数量，便于 review 发现异常漂移。

### 目标、CTest 和 label 合同

| 维度 | 格式 | 示例 |
|---|---|---|
| CMake executable target | `1q_<domain>_<type>_tests` | `1q_sar_replay_tests` |
| CMake aggregate target | `1q_<type>_tests` | `1q_unit_tests`（custom target） |
| 全测试 aggregate | `1q_tests` | 依赖全部启用的测试分区 |
| CTest name | `<type>::<domain>` | `replay::sar` |
| 类型 label | `<type>` | `unit`、`replay` |
| 业务域 label | `<domain>` | `sar`、`airborne_radar` |
| 执行策略 label | `ci_required`/`ci_advisory`/`fast`/`slow`/`known_limit` | 与类型正交 |

典型运行方式：

```bash
ctest --test-dir build/llvm-ninja-debug-local -L unit --output-on-failure
ctest --test-dir build/llvm-ninja-debug-local -L unit -L sar --output-on-failure
ctest --test-dir build/llvm-ninja-debug-local -L replay -L airborne_radar --output-on-failure
ctest --test-dir build/llvm-ninja-debug-local -L ci_required --output-on-failure
./build/llvm-ninja-debug-local/bin/1q_sar_unit_tests --gtest_filter='SarGeometryTest.*'
```

不在首批使用 `gtest_discover_tests()`：当前 case 数量大，参数化测试较多，全部展开会显著增加 configure/test discovery 和 CI 调度成本。后续只有在“单 case 重试/超时/资源锁”确有需求时，再用小范围实验评估 `DISCOVERY_MODE PRE_TEST`。

## 现有测试的首轮归位

| 当前测试 | 目标归属 | 处理原则 |
|---|---|---|
| `ar/eos/esr/sbirs/sar_*` 局部算法/validation/resolver | `unit/<owner>` | 保留 suite/case 和断言，仅移动路径 |
| `fd_*` | `unit/flight_dynamic` 或按行为进入 `integration/flight_dynamic`、`performance/flight_dynamic` | 先按文件审计，不按前缀机械归类 |
| coordinate/estimation/numeric/standard atmosphere | `unit/common` | 归产品 common owner |
| `json_reader_test.cpp` | `unit/examples` | 继续显式加入 `examples/common/json_reader.cpp` 作为 target source |
| replay trace writer/compression | `replay/common` | 从 unit 目标移除，消除重复编译 |
| 各模块 codec roundtrip/replay session/trace adapter | `replay/<owner>` | 每个源只进入 replay 分区 |
| `*_session_test.cpp` | `integration/<owner>` | 当前五个 integration 文件直接归位 |
| `multi_model_scenario_test.cpp` | `integration/cross_domain` | 跨域场景独立分区 |
| `sar_session_pipeline_test.cpp` | 候选 `integration/sar` | 逐 case 审计；局部算法 case 留 unit，完整 session case 移 integration |
| `sar_cycle_input_adapter_bridge_test.cpp` | 候选 `integration/sar` | 根据是否跨 public input → adapter → runtime 链路决定 |
| public convenience/header smoke | `contract/<owner>` 或 `contract/public_api` | 编译型 contract 分区 |
| include direction/style、CMake/docs/layout guards | `contract/<owner|project>` | 由 `ScriptGuards.cmake` 通用注册 |
| public header/SAR C++11 probes | `compatibility/<owner>` | 从 contract/SAR 专项文件中移出 |
| SAR FFT performance | `performance/sar` | 保持阈值和 release 运行要求不变 |
| installed consumers | `consumer` | 保持独立工程和 CI 安装验证 |

## SAR 和 flight_dynamic 的处理

### SAR

最终删除 `SarTests.cmake`。SAR 不再通过 suite 名称维护一条巨型 unit filter，而是自然形成以下分区：

- `unit::sar`
- `integration::sar`
- `replay::sar`
- `contract::sar`
- `performance::sar`
- `compatibility::sar`

`sar_frozen_sources` 和 `sar_doc_governance_guard` 仍可保留 SAR domain label，但它们属于 contract/governance；`sar_cxx11_compat` 属于 compatibility。迁移期间可以暂时附加旧 label `sar_ci`，CI 切换到 `ci_required` 后删除旧别名。

### flight_dynamic

FD 的可选 JSBSim 依赖保留，但五层手写 filter 只作为过渡实现。长期按测试源的行为职责拆成：

- smoke/contract/controllability：放入 unit 或 integration 分区，再用执行策略 label 标记；
- performance：进入 `performance/flight_dynamic`；
- known-limit：保留真实断言，附加 `known_limit`，不进入绿色硬门禁。

若一个现有 `.cpp` 同时包含多个 tier，先在不改断言的前提下按 fixture/场景拆文件，再删除对应 filter。不得通过放宽断言、增加 skip 或调低阈值来完成架构迁移。

## 分阶段实施计划

### Phase 0：冻结基线和自动守护 ✅

修改范围：只新增 registry/layout guard 和基线清单，不移动测试源、不改变 target。

执行项：

1. fresh configure 后记录当前 `_test.cpp` 集合、每个可执行文件的源集合、CTest 名称/label、GoogleTest 数量。
2. 新增 registry 完整性检查：测试源无 orphan、无重复注册；允许当前已知 replay 重复以显式 allowlist 暂时通过。
3. 新增 `test_layout_guard`，首批只禁止 `tests/` 根目录新增 `_test.cpp`，不立即拒绝当前平铺结构。
4. 将现有 provisional split 标记为 transition，不再继续扩充 `SarTests.cmake` filter。

基线快照（FD=OFF，2026-07-11）：

- 全量 `_test.cpp`：178（unit）+ 5（integration）+ 8（contract compiled）+ 1（performance）= 192 个唯一源；
  另有 9 个 consumer 源（独立工程，不计入 partition registry）。
- registry 统计（`oneq_finalize_test_registry()` 输出）：198 registered，其中
  unit=165、fd=8、performance=1、integration=5、contract_compiled=8、replay_fast=11。
  replay_fast 与 unit 存在 11 个重叠，已由 `ONEQ_TEST_OVERLAP_ALLOWLIST` 显式登记，待 Phase 3 消除。
- CTest 清单：30 项（FD=OFF）。contract 标签 21 项（原 20 + test_layout_guard）；
  sar_ci 标签 7 项；fd_ci 标签 0 项（FD=OFF）。
- 交付物：`tests/cmake/TestRegistry.cmake`、`tests/contract/check_test_layout.cmake`、
  `tests/CMakeLists.txt` 头注释标注 provisional split 为 transition。

退出条件：默认/FD=ON fresh configure、当前 CTest 清单、contract 20 项和 CI 绿色 9 项保持不变。

验证结果：

- `cmake --fresh --preset llvm-ninja-debug-local`：通过；registry 输出 198 sources、无 orphan/duplicate。
- FD=ON fresh configure（`-D ONEQ_ENABLE_FLIGHT_DYNAMIC=ON`）：通过；fd 分区 8 sources 存在，registry 总数不变。
- `ctest ... -j 4`（FD=OFF）：30/30 通过，100%。

回滚边界：可独立删除新 guard/registry，不触及测试代码。

### Phase 1：建立类型 × 域目录，不改变测试语义 ✅

按 owner 小批迁移，每批只移动路径和修正 source discovery：

1. common/examples；
2. EOS/SBIRS；
3. ESR/AR；
4. SAR；
5. FD；
6. integration/contract/performance/compatibility/consumer。

每批使用 Git rename 保留历史；不重命名 suite/case，不改变断言。当前 unit/replay 重叠先由 registry 明示，等 Phase 3 再消除。

迁移结果（2026-07-11）：

- `tests/unit/` → `common/`(8) `examples/`(1) `airborne_radar/`(41) `electronic_surveillance_radar/`(18) `electro_optical_sensor/`(19) `sbirs_sensor/`(17) `sar/`(66) `flight_dynamic/`(8)，共 178 文件，0 flat。
- `tests/integration/` → `airborne_radar/` `electronic_surveillance_radar/` `electro_optical_sensor/` `sbirs_sensor/` `cross_domain/`，共 5 文件。
- `tests/contract/` compiled → `public_api/` `airborne_radar/`(2) `electronic_surveillance_radar/`(2) `electro_optical_sensor/` `sar/` `sbirs_sensor/`，共 8 文件；`.cmake` script guards 暂留 contract 根目录，Phase 3 由 ScriptGuards.cmake 统一归类。
- `tests/performance/` → `sar/`，1 文件。
- `tests/consumer/` 保持单层（消费者类型无 domain 子目录）。
- `TestTargets.cmake`/`TestRegistry.cmake` 中的硬编码路径同步更新到新 `unit/<domain>/` 位置。

退出条件：所有 `_test.cpp` 位于允许的类型/域路径；源码集合与 Phase 0 基线完全一致；现有 target、CTest 名称和运行结果不变。

验证结果：

- `cmake --preset llvm-ninja-debug-local`：通过；registry 输出与 Phase 0 完全一致（198 sources，partition 计数不变）。
- `cmake --build ... --target 1q_unit_tests 1q_contract_tests 1q_integration_tests 1q_replay_fast_tests 1q_performance_tests`：通过。
- `ctest ... -j 4`（FD=OFF）：30/30 通过。

回滚边界：每个 owner 一次独立提交，可按模块回退。

### Phase 2：拆分 unit 分区目标

1. 新增通用 `oneq_add_test_partition()` 和 `partitions/Unit.cmake`。
2. 依次从 legacy `1q_unit_tests` 移出 common/examples、EOS/SBIRS、ESR/AR、SAR、FD 源并创建模块目标。
3. 每迁一个 owner，比较 legacy+新目标测试总数、focused 用例和构建时间；registry 保证源只归属一个目标。
4. 所有 owner 迁完后，把 `1q_unit_tests` 从 executable 改为依赖全部 unit 分区的 custom aggregate target。
5. 同批更新 `generate_maneuver_plots.sh`，改为运行 `1q_flight_dynamic_unit_tests` 或对应 FD 分区。

退出条件：`cmake --build ... --target 1q_unit_tests` 仍有效；`ctest -L unit` 覆盖全部 unit 分区；无旧 `bin/1q_unit_tests` 的活跃调用者。

回滚边界：在最后切换 aggregate target 前，legacy executable 始终保留未迁移源；可按 owner 回退。

### Phase 3：拆分 replay、integration、contract、performance、compatibility

1. 将 11 个 replay-fast 源归入 `replay/<owner>`，删除 6 个重复编译关系；`fast` 作为 label 保留，不再作为 target 类型名。
2. 为现有五个 integration 文件建立 owner 分区；审计并拆分 SAR session/pipeline 的 unit/integration 混合语义。
3. 将 compiled contract 按 owner/public_api 分区；script guards 统一由 `ScriptGuards.cmake` 注册。
4. 将 C++11 探针迁到 compatibility；performance 按 domain 注册。
5. 将 `1q_replay_fast_tests`、`1q_integration_tests`、`1q_contract_tests`、`1q_performance_tests` 变为 aggregate build targets。

退出条件：每个 `_test.cpp` 恰好属于一个编译目标；SAR 不再借 unit/replay filter 伪装 integration；contract 与 compatibility 类型边界清晰。

回滚边界：按测试类型独立提交，先保留旧 CTest label 别名，目标切换失败可回退单一类型。

### Phase 4：消除专项 filter 和迁移 CI 策略

1. 删除 `SarTests.cmake`，把 SAR label 映射并入通用 registry。
2. 按文件/fixture 拆分 FD tier，逐项删除 `FlightDynamicTests.cmake` 的手写 filter。
3. 为当前绿色门禁中的等价分区附加 `ci_required`；unit advisory 使用 `ci_advisory`；performance/known-limit 保持非阻断。
4. 将 CI 的 `-L 'sar_ci|integration|replay_fast'` 改为 `-L ci_required`，确认测试集合在切换前后等价后再删除旧 label。

退出条件：业务模块新增测试不需要编辑 suite/case filter；CI 选择只依赖通用策略 label；阻断范围没有静默扩大或缩小。

回滚边界：旧 label 与新 label 至少并存一个验证批次，CI 可单独回退。

### Phase 5：覆盖率、工具和活跃文档收口

覆盖率是目标拆分的最高风险兼容面，必须先做实验再删除旧主映射二进制：

1. 在 coverage preset 下评估专用 `1q_coverage_tests`/coverage mapping runner；它可聚合全部测试源，但不进入普通 Debug/Release 构建。
2. 比较拆分前后的 region/function/line 总数和已知 0% 文件，确认没有因多二进制 mapping hash 冲突产生假回归。
3. `coverage_report.sh` 改为从 CTest/registry 或 `bin/1q_*_tests` 发现实际分区，不维护固定六项列表。
4. 更新 `cmake/README.md`、`tests/README.md`、`docs/practice/ci.md`、`docs/practice/coverage.md`；历史 review 不改写结果。

若 coverage runner 不能证明映射等价，则暂缓删除 coverage preset 下的 monolithic runner，但普通构建仍使用分区目标；不得接受虚假覆盖率下降。

退出条件：全量和按 label 覆盖率均可生成；活跃文档没有旧二进制路径；工具不依赖硬编码模块清单。

### Phase 6：全矩阵验收和旧结构删除

1. 删除 provisional `TestTargets.cmake`、`SarTests.cmake`、`FlightDynamicTests.cmake` 中已被 registry/partitions 取代的逻辑。
2. 收紧 `test_layout_guard`，拒绝平铺源、未知 domain、重复归属和新手写 domain filter。
3. 运行完整验证矩阵并记录 test count、构建时间和 coverage 差异。
4. 一个稳定 CI 周期后删除 `sar_ci`、`replay_fast`、`fd_ci` 等过渡 label。

## 验证矩阵

| 范围 | 命令/检查 | 验收 |
|---|---|---|
| 配置 | `cmake --fresh --preset llvm-ninja-debug` | registry 无 orphan/duplicate；输出分区统计 |
| FD 配置 | FD=ON 的 fresh configure | flight_dynamic 分区和 labels 存在 |
| 构建兼容 | `cmake --build --preset llvm-ninja-debug --target 1q_unit_tests 1q_contract_tests 1q_tests` | aggregate target 可用 |
| unit | `ctest ... -L unit` | 全部分区通过；总 case 不减少 |
| 单域 | `ctest ... -L unit -L sar` 等 | 每个 owner 可独立运行 |
| replay | `ctest ... -L replay` | roundtrip/divergence/trace 集合完整且不重复编译 |
| integration | `ctest ... -L integration` | 五个现有场景 + 经审计的 SAR 场景 |
| contract | `ctest ... -L contract` | 20 项当前基线及新 layout guard 通过 |
| compatibility | `ctest ... -L compatibility` | public/SAR C++11 探针通过 |
| CI gate | 切换前后分别列出 `ci_required` 测试名 | 集合等价、9/9 当前绿色基线不缩水 |
| consumer | 安装后构建并运行九个 consumer | `find_package(1q)` 行为不受影响 |
| coverage | 全量与 `--label unit`/`--label sar` | mapping 无假 0%、指标可解释 |
| Release/performance | Release preset `-L performance` | 阈值和超时不变 |

每批必须同时比较：

- `_test.cpp` 文件集合；
- registry 中的唯一归属集合；
- CTest 名称和 labels；
- GoogleTest 实际 case 数；
- 失败/skip/known-limit 数量；
- 编译耗时与重复编译源数量。

## 主要风险与控制

| 风险 | 控制方式 |
|---|---|
| 按文件前缀误判测试类型 | 逐文件读取 fixture/依赖/断言；prefix 只用于初筛 |
| 目录移动和目标拆分同时导致丢测试 | Phase 1 只移动，Phase 2/3 才改 target；registry 做集合差异检查 |
| module target 增多导致链接时间上升 | 记录基线并按 owner 批量，不拆到单文件/单 fixture 目标 |
| CI label 切换静默改变硬门禁 | 新旧 label 并存一批，比较 CTest 名称集合后切换 |
| coverage 主映射二进制消失 | coverage preset 专用 runner 实验通过后再删除旧路径 |
| FD known-limit 被错误纳入绿色门禁 | `known_limit` 作为独立策略 label，保持现有断言和非阻断语义 |
| CMake 再次按模块形成专项文件 | partitions 只按测试类型拆，所有 domain 走同一个注册 API |

## 完成定义

满足以下条件才算架构迁移完成：

1. 测试作者只需选择“类型/域”目录并新增 `_test.cpp`，无需编辑远端 suite filter。
2. 每个测试源只有一个 owner、一个测试类型、一个编译分区。
3. 任意业务域都能用 CTest label 独立运行，不只 SAR。
4. `SarTests.cmake` 和 FD 巨型 filter 被删除，模块特例只剩必要的 target dependency。
5. 旧 aggregate build target 可用，活跃脚本不依赖旧固定二进制路径。
6. CI、coverage、consumer、contract、compatibility 和 performance 验收均有可重复命令。
7. 没有通过减少测试、增加 skip、放宽断言或降低阈值获得绿色结果。
