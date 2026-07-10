# 代码覆盖率测量指南

Status: active
Last-reviewed: 2026-07-02
Authority: build infrastructure

本项目使用 **LLVM source-based coverage**（`-fprofile-instr-generate -fcoverage-mapping`）测量代码覆盖率。它基于 LLVM 源码 region，分支覆盖率精度高于传统 gcov，是 Clang/Apple 官方推荐路径。

## 快速开始

```bash
# 0. 生成 Conan toolchain（首次或依赖变更后）
bash scripts/bootstrap_conan.sh llvm-ninja-coverage

# 1. 配置（使用专用 coverage preset，已设 ENABLE_COVERAGE=ON）
cmake --preset llvm-ninja-coverage

# 2. 编译
cmake --build --preset llvm-ninja-coverage

# 3. 跑测试并生成覆盖率报告
./tools/coverage_report.sh
```

脚本结束后会在终端打印顶层覆盖率摘要，HTML 报告位于 `build/llvm-ninja-coverage-local/coverage_report/html/index.html`。

打开报告：

```bash
./tools/coverage_report.sh --open
```

## 按测试子集生成覆盖率

coverage_report.sh 支持 `--label` 透传给 ctest，与项目现有的 CTest label 体系对齐：

```bash
./tools/coverage_report.sh --label unit       # 仅单元测试
./tools/coverage_report.sh --label sar_ci      # SAR CI 子集
./tools/coverage_report.sh --label fd_smoke    # flight_dynamic 冒烟层
./tools/coverage_report.sh --label contract    # 契约/架构守护
```

可用于定位"某个模块的覆盖率缺口集中在哪些测试层"。

## 覆盖率指标解读

llvm-cov 报告四类指标：

| 指标 | 含义 | 本项目建议 |
|---|---|---|
| **Region** | 源码 region（表达式/语句块）是否执行 | 次要参考 |
| **Branch** | 分支（if/else/三元/逻辑运算）各方向是否走到 | **主指标** |
| **Function** | 函数是否被调用过 | 基础参考 |
| **Line** | 行是否执行 | 辅助参考 |

**为什么以 Branch（分支覆盖率）为主指标？** 本项目是雷达信号/电子侦察仿真系统，数值密集、分支条件多（PRF 退化、几何边界、状态机转换）。行覆盖率可能 100% 但某个 `else` 分支从未走过；分支覆盖率才能暴露这类隐藏路径。这也是数值算法代码最常见的 bug 藏匿点。

## 如何阅读 HTML 报告

打开 `coverage_report/html/index.html` 后：
- 顶层是目录树，按 `src/<module>/` 展开
- 每个源文件可点击进入逐行视图：绿色 = 已覆盖，红色 = 未覆盖，黄色 = 部分分支覆盖
- 顶部可切换 Region/Branch/Function/Line 视角

定位补测优先级时，建议排序：**未覆盖分支密度高的文件 > 未覆盖函数 > 未覆盖行**。

## 与现有测试体系的关系

覆盖率回答"**测得全不全**"（广度），不回答"**测得对不对**"（深度）。它和现有体系互补：

| 已有的 | 覆盖率新增的 |
|---|---|
| 单元/集成/契约/性能分层测试 | 量化各层实际触达的代码区域 |
| contract 守护脚本（架构规则） | 测量守护脚本无法覆盖的数值逻辑 |
| evidence-first 证据矩阵 | 给证据矩阵补上"覆盖盲区"维度 |

**覆盖率不是 KPI，是诊断工具。** 100% 覆盖率不等于无 bug，但 < 50% 覆盖率几乎必然意味着未测试区域有未知缺陷。建议用法：
1. 先跑出全量基线数字（首次知道"我们到底覆盖了多少"）
2. 针对低覆盖模块（尤其 `sar/imaging`、`*/session`、数值边界路径）定向补测
3. 重构前后对比覆盖率曲线，防止回归

## 工作原理

```
源码 + -fcoverage-mapping
        ↓ 编译
插桩二进制 (.o / 可执行)
        ↓ 运行测试 (LLVM_PROFILE_FILE 指定输出)
.profraw (运行时计数器)
        ↓ llvm-profdata merge -sparse
.profdata (合并后剖面)
        ↓ llvm-cov show/report/export
HTML / 文本摘要 / JSON
```

## 局限性

- **仅 Clang**：source-based coverage 依赖 LLVM。MSVC/GCC 构建无法使用本 preset。本项目 `llvm-ninja-*` preset 天然满足。
- **编译开销**：插桩会增加约 20-30% 编译时间与产物体积，且禁用部分优化。所以 coverage 是独立 preset，不影响 `llvm-ninja-debug` / `llvm-ninja-release`。
- **测试自身也被插桩**：测试代码也会出现在原始数据中，但 `coverage_report.sh` 仅统计 `src/` 与 `include/` 路径，自动排除测试目录。
- **第三方不参与**：JSBSim、gtest 等通过 Conan 预编译或独立链接，不重新编译，故不在覆盖率统计范围内（符合预期——我们只关心自有代码）。

## 故障排查

**"未找到 .profraw 文件"**
→ 确认用了 `llvm-ninja-coverage` preset（而非普通 debug），且 `ENABLE_COVERAGE` 在 CMakeCache 中为 ON。

**"部分测试失败，覆盖率数据可能不完整"**
→ 这是警告而非错误。失败的测试不贡献数据，但已通过的测试仍会生成报告。修复失败测试后重跑即可。

**报告里文件太多/想聚焦某个模块**
→ `llvm-cov` 的目录参数由脚本自动从 `src/`+`include/` 收集。如需聚焦，可手动编辑脚本或用 `--label` 缩小测试范围。
