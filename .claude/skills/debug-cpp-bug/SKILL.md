---
name: debug-cpp-bug
description: >
  1Q 项目的系统性 C++ bug 调试工作流。当用户提到调试、debug、修 bug、crash、
  segfault、测试失败、断言失败、结果不对、异常行为、core dump、ASAN 报错时触发。
  也适用于用户说"这段代码为什么不工作"、"帮我找找问题"、"挂了"、"崩了"、
  "跑不过"、"数据不对"等场景。涵盖复现、日志定位（spdlog）、LLDB 交互调试、
  GTest 用例隔离、最小化修复和回归验证的完整流程。
---

# Bug 调试工作流

按阶段顺序推进，根据 bug 复杂度可跳过已完成的阶段。

## 阶段 1：复现

目标是让 bug 可靠地复现，抓住它。

1. **确认构建 preset**
   - 调试用 `llvm-ninja-debug-local`（带调试符号，无优化）
   - 如果 bug 只在 release 出现，用 `llvm-ninja-release-local`

2. **构建并复现**
   ```bash
   preset="llvm-ninja-debug-local"
   cmake --preset "$preset" >/tmp/1q-cmake.log 2>&1 || { tail -n 80 /tmp/1q-cmake.log; false; }
   cmake --build --preset "$preset" >/tmp/1q-build.log 2>&1 || { tail -n 80 /tmp/1q-build.log; false; }
   ```

3. **如果用户给了具体的测试用例**，直接跑：
   ```bash
   ctest --preset "$preset" --output-on-failure -R <test_name>
   ```

4. **如果没有现成测试**，尝试从用户描述中提取复现步骤。必要时写一个最小复现测试。

5. **记录现象**：crash 时的信号类型、错误日志、断言消息、错误的数据值。这些是后续定位的线索。

## 阶段 2：定位

目标是缩小 bug 的代码范围。

### 2a. 日志定位（spdlog）

日志适合观察多帧循环、异步逻辑、不方便断点的场景。

- 在可疑路径上添加 `spdlog::debug/info`，打印关键变量值
- flow 路径用 `spdlog::debug`，关键状态变化用 `spdlog::info`，failure 用 `spdlog::error`
- 添加日志时注意：不要改变程序行为（例如别在日志表达式里调用有副作用的函数）

典型模式：
```cpp
spdlog::debug("Maneuver::update: state={}, heading={:.1f}, target_bearing={:.1f}",
              static_cast<int>(state_), heading, target_bearing);
```

### 2b. 交互式调试（LLDB）

LLDB 适合精确定位：看调用栈、检查变量、设条件断点。

启动方式（以测试二进制为例）：
```bash
# 先找到测试二进制路径
find build -name "<test_name>" -type f

# 启动 LLDB
lldb -- ./<test_binary> --gtest_filter=<TestSuite.TestName>
```

常用操作：
- `bt` — 查看崩溃时的调用栈
- `b file.cpp:42` — 在指定行设断点
- `b file.cpp:42 -c "x > 5"` — 条件断点，只在条件满足时停下
- `p var` — 打印变量值
- `n` / `s` — 单步（next 不进入函数 / step 进入函数）
- `c` — 继续运行
- `frame variable` — 查看当前栈帧所有局部变量

### 2c. 结合测试定位

用 GTest filter 跑单个用例来隔离问题：
```bash
ctest --preset "$preset" --output-on-failure -R <pattern>
```

如果某个用例耗时很长，先用 `--gtest_filter` 缩小到最小失败用例。

### 定位判断

根据线索判断 bug 类型，选择对应策略：

| 现象 | 类型 | 策略 |
|------|------|------|
| SIGSEGV / SIGABRT | 内存错误 | LLDB 看调用栈，检查指针和数组越界 |
| 断言失败 | 前置条件违反 | 看断言条件，追踪谁破坏了不变量 |
| 数值不对 | 逻辑错误 | spdlog 打印中间值，LLDB 条件断点 |
| 偶发失败 | 竞争/未初始化 | 检查默认初始化、时序依赖 |
| 只在 release 出现 | 未定义行为 | 开 ASAN (`-fsanitize=address`) |

## 阶段 3：修复

- **最小化变更**：只改需要改的，不做附带重构
- **不改无关代码**：项目约定不 reformat 未触碰的代码
- **不改公共 API**：优先改 `src/` 内部实现，避免扩大 `include/` 变更
- **不引入异常**：项目禁止 C++ 异常
- 添加注释仅当 WHY 不明显时（如隐含约束、微妙的不变量）

## 阶段 4：验证

1. **原测试通过**：确保触发 bug 的测试现在通过
2. **回归测试**：跑全量测试确认没有引入新问题
   ```bash
   ctest --preset "$preset" --output-on-failure -j 4
   ```
3. **新增测试**：如果修复涉及公共 API 或重要逻辑，添加测试覆盖修复场景
4. **清理调试日志**：移除修复过程中添加的临时 spdlog 调试日志（保留有长期价值的日志）

## 快速参考

项目构建 preset：
- `llvm-ninja-debug-local` — 日常调试
- `llvm-ninja-release-local` — release 复现
- `llvm-ninja-release-local-stress` — 压力测试

日志前缀：`/tmp/1q`
