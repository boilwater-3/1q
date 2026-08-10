---
name: test-coverage-strategy
description: Use when writing tests, evaluating test coverage, choosing test methods, or when GTest/GMock cannot cover certain scenarios (statistical correctness, state-space explosion, long-cycle accumulation, external boundary failures). Covers the seven-layer supplementary testing strategy for this project.
argument-hint: "[scenario description or test gap]"
allowed-tools:
  - Bash
  - Read
  - Grep
  - Agent
---

# 测试分层覆盖策略

当 GTest/GMock 单进程、确定性、手写用例的框架能力不足以覆盖某个场景时，按本文的决策流程选择补充层。各层的完整原理和配置细节见对应的参考文档。

## GTest/GMock 能力边界

| 盲区 | 为什么 GTest 覆盖不了 | 本项目典型实例 |
|---|---|---|
| **统计/概率正确性** | 单次 `EXPECT_GT(snr, threshold)` 不能证明 Pd/Pfa 分布正确 | AR 检测概率链、ESR 截获概率、IMM 多模型转移 |
| **状态空间爆炸** | 手写用例只能覆盖有限路径组合 | AR 四子系统交互、7 种 abort reason 与恢复路径组合 |
| **长周期累积效应** | 单测只跑 1-2 周期，看不出数值漂移、资源泄漏 | 浮点误差跨 1000 周期累积、track lifecycle 跨周期退化 |
| **外部边界故障** | Mock 只能模拟"正常返回"，不能模拟真实 I/O 损坏 | HDF5 文件截断、JSBSim 异常穿透、FlatBuffers schema 版本不匹配 |

## 决策流程

面对一个 GTest 难以覆盖的测试场景时，按以下流程选择补充方法：

| 场景特征 | 补充层 | 触发条件 |
|---|---|---|
| 参数组合太多，手写枚举不完 | **Layer 2: batch sweep** | 多个独立参数的交叉组合 |
| 需要验证跨多周期的完整路径 | **Layer 1: replay trace** | 涉及 failure→recovery、patch 提交/回滚 |
| 结构/命名/架构规则约束 | **Layer 3: contract guard** | 编译前就能静态判定的约束 |
| 长周期数值漂移 | **Layer 1: replay trace** | 1000+ 周期累积误差 |
| 统计分布正确性 | **Layer 7: property-based** | Pd/Pfa、蒙特卡洛、随机过程 |
| 未定义行为/内存安全 | **Layer 5: sanitizers** | GTest 全绿但怀疑 UB |
| 未知非法输入边界 | **Layer 6: fuzzing** | 解析器、codec、字节流入口 |
| 不确定盲区在哪 | **Layer 4: coverage** | 先跑覆盖率报告定位 |

## 已有层（可直接使用）

### Layer 1: 确定性重放 (Replay Trace) ✅

跑生产代码 → 记录 trace → 重放逐周期比较。divergence 自动发现。

- 覆盖: AR/ESR/EOS/SAR/SBIRS 五模块，failure marker 三域对齐
- 命令: `ctest --preset llvm-ninja-debug-local -L replay --output-on-failure`
- 新增路径: 优先在对应 batch validation sequence 中添加场景
- 详阅: `docs/common/contract.md` §Replay 与 trace 语义

### Layer 2: 批量场景验证 (Batch Validation) ✅

通过 public Session API 驱动多周期仿真，结构化 checks 验证。230 个场景 (199 sweep + 31 sequence)。

- 构建: `cmake --preset llvm-ninja-release-local && cmake --build --preset llvm-ninja-release-local --target ar_batch_validation ...`（随测试构建纳入，见 `tests/consumer/batch_validation/README.md`）
- 运行: `./build/llvm-ninja-release-local/bin/<domain>_batch_validation --suite all`
- 详阅: `docs/practice/batch_validation.md`

### Layer 3: 契约守护 (Contract Guards) ✅

CMake configure 时静态检查结构约束（include 方向、命名、public API 边界）。检查失败阻止 configure。

- 命令: `cmake --preset llvm-ninja-debug-local`（configure 时自动运行）
- 详阅: `docs/common/contract.md` §测试架构

### Layer 4: 覆盖率诊断 (LLVM Source-based Coverage) ✅

分支覆盖率是主指标。发现盲区，定向补测。

- 命令: `bash scripts/bootstrap_conan.sh llvm-ninja-coverage && cmake --preset llvm-ninja-coverage && cmake --build --preset llvm-ninja-coverage && ./tools/coverage_report.sh`
- 报告: `./tools/coverage_report.sh --open` 打开 HTML，按未覆盖分支密度排序文件
- 详阅: `docs/practice/coverage.md`

## 待建设层

以下三层当前未落地，Skill 中保留设计蓝图供需要时参考实现：

### Layer 5: Sanitizers

新增 CMake preset: UBSan (`-fsanitize=undefined`)、ASan (`-fsanitize=address`)、TSan (`-fsanitize=thread`)。按性能开销从低到高引入。每个 preset 继承 `llvm-ninja-debug-local`，只添加对应 flag。

### Layer 6: Fuzzing

为 FlatBuffers codec、JSON 解析、trace 文件读取等字节流边界添加 libFuzzer target。每个 target 只验证"任意输入 → 不崩溃不泄漏不部分修改"。

### Layer 7: Property-based Testing

对物理公式不变量、坐标变换往返恒等性、统计算子无偏性，引入 [rapidcheck](https://github.com/emil-e/rapidcheck) (header-only，与 GTest 集成)。在引入前，用 GTest 循环随机输入作为替代方案。

## 覆盖率驱动的补测工作流

1. **基线**: `./tools/coverage_report.sh` 生成全量 HTML 报告
2. **定位**: 展开 `src/<module>/`，标记红色（未覆盖）和黄色（部分分支覆盖）文件
3. **分类**: 红色函数→GTest 单测、红色分支→边界+属性测试、红色文件→batch validation、红色跨周期→replay trace
4. **验证**: 补测后重跑覆盖率，对比分支覆盖率变化
5. **守护**: 覆盖率是诊断工具，不作为 CI 阻断门禁

## 关键命令速查

```bash
# 单元测试
ctest --preset llvm-ninja-debug-local -L unit --output-on-failure -j 4

# 集成测试
ctest --preset llvm-ninja-debug-local -L integration --output-on-failure

# Replay 测试
ctest --preset llvm-ninja-debug-local -L replay --output-on-failure

# 契约测试
ctest --preset llvm-ninja-debug-local -L contract --output-on-failure

# PR 关键路径
ctest --preset llvm-ninja-debug-local -L ci_required --output-on-failure -j 4

# Batch validation（tests/consumer/batch_validation，随测试构建纳入）
cmake --preset llvm-ninja-release-local
cmake --build --preset llvm-ninja-release-local --target \
  ar_batch_validation eos_batch_validation esr_batch_validation \
  sar_batch_validation sbirs_batch_validation -j 4
./build/llvm-ninja-release-local/bin/ar_batch_validation --suite all

# 覆盖率
bash scripts/bootstrap_conan.sh llvm-ninja-coverage
cmake --preset llvm-ninja-coverage && cmake --build --preset llvm-ninja-coverage
./tools/coverage_report.sh && ./tools/coverage_report.sh --open
```

## 参考文档

- 公共契约与测试架构: `docs/common/contract.md` §测试架构
- 覆盖率测量: `docs/practice/coverage.md`
- 批量场景验证: `docs/practice/batch_validation.md`
- 测试目录约定: `tests/README.md`
- 各模块设计: `docs/<module>/design.md`
