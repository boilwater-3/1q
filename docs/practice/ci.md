# CI 持续集成

Status: active
Last-reviewed: 2026-07-02
Authority: build & test infrastructure

本仓库的持续集成（CI）由两个 GitHub Actions workflow 组成，按"快速门禁 vs 慢任务"分层。

## Job 矩阵总览

| Workflow | 触发 | Job | 内容 | 阻断 PR |
|---|---|---|---|---|
| `ci.yml` | push / PR | **guard** | `-L contract`：17 个架构守护 + 契约测试，编译前先跑 | ✅ |
| `ci.yml` | push / PR | **build-test** | Debug 构建 + 绿色门禁（`sar_ci`/`integration`/`replay_fast`）+ unit advisory | ✅（unit 不阻断） |
| `nightly.yml` | cron 02:00 + 手动 | **performance** | Release 构建 + 性能/cxx11 兼容 gate | ❌ |
| `nightly.yml` | cron + 手动 | **flight-dynamic** | FD=ON + JSBSim 数据 + `fd_ci` | ❌ |
| `nightly.yml` | cron + 手动 | **coverage** | 插桩构建 + 覆盖率报告 artifact | ❌ |

设计原则：**CI 一上线就必须是绿的。** 只把当前已知健康的 label 作为硬门禁，已知失败的 `unit` 降级为 advisory（warn 不阻断）。

## 为什么 unit 是 advisory

`unit` label 当前有 2 个确定性失败（`sar_session_config_builder_test.cpp` 的 `PassesOnHealthyBuiltConfig` / `BuildsSessionAndReportsNoIssuesForHealthyConfig`），根因是 commit `545ac428` 新增了跨字段物理校验 `kSampleWindowTooSmallForPulse`（距离采样窗口必须容纳完整脉冲宽度），但 `SarSessionConfigBuilder` 的 `kStripmapSurvey` 默认值不满足该约束。

这是**测试需更新**（调整默认 config 使其物理自洽），不是产品代码 bug。修复后应将 `unit` 从 advisory 移入 `build-test` 的硬门禁——届时删除 ci.yml 中 `unit advisory` step 的 `continue-on-error: true` 即可。

## 平台范围

当前仅 macOS（`macos-14` arm64 runner）。理由：
- macOS 路径下 Conan 提供全部依赖（含 JSBSim 预编译包 `jsbsim/1.3.1`），最干净可靠。
- Windows conan 路径的 JSBSim 来源存在未决问题（`conanfile.py` 在 Windows 下不 `requires` jsbsim，但 `ProjectDependencies.cmake` 仍 `find_package`），需先核实再启用。

## 本地复现

CI 跑的每条命令都可在本地复现，便于调试失败：

```bash
# === guard job（contract 守护）===
cmake --preset llvm-ninja-debug
ctest --test-dir build/llvm-ninja-debug-local -L contract --output-on-failure -j 4

# === build-test job（绿色门禁）===
cmake --preset llvm-ninja-debug
cmake --build --preset llvm-ninja-debug -j 4
ctest --test-dir build/llvm-ninja-debug-local -L 'sar_ci|integration|replay_fast' --output-on-failure -j 4

# === build-test 的 unit advisory ===
ctest --test-dir build/llvm-ninja-debug-local -L unit --output-on-failure -j 4

# === nightly: performance ===
cmake --preset llvm-ninja-release
cmake --build --preset llvm-ninja-release -j 4
ctest --test-dir build/llvm-ninja-release -L 'performance|sar_performance' --output-on-failure -j 4
ctest --test-dir build/llvm-ninja-release -R sar_cxx11_compat --output-on-failure -j 4

# === nightly: flight-dynamic（需先获取 JSBSim 数据）===
git clone --depth 1 https://github.com/JSBSim-Team/jsbsim.git third_party/jsbsim
cmake --preset llvm-ninja-debug -D ONEQ_ENABLE_FLIGHT_DYNAMIC=ON
cmake --build --preset llvm-ninja-debug -j 4
ctest --test-dir build/llvm-ninja-debug-local -L fd_ci --output-on-failure -j 4

# === nightly: coverage ===
cmake --preset llvm-ninja-coverage
cmake --build --preset llvm-ninja-coverage -j 4
bash tools/coverage_report.sh
```

## 手动触发 nightly

GitHub 仓库 → **Actions** 标签页 → 左侧选 **Nightly** → 右上角 **Run workflow**。

适用于：修复后想立即验证、不想等到凌晨 2 点自动触发。

## 扩展指南

**新增 Windows job**：在 `nightly.yml` 加一个 `runs-on: windows-latest` 的 job，用 `windows-vs2019` preset。但必须先解决 JSBSim 在 Windows conan 路径下的来源问题（核实 `conanfile.py` 是否需要为 Windows 补 `jsbsim` requires，或改走源码编译路径）。

**把 unit 升级为硬门禁**：修复 2 个 SAR 失败 case 后，在 `ci.yml` 的 `build-test` job 里：
1. 删除 `unit advisory` 整个 step
2. 在 `Run green-label tests` 的 `-L` 参数里追加 `|unit`

**新增 lint job**（clang-format/tidy）：在 `ci.yml` 加独立 job 跑 `cmake --build --target format-check`（需 runner 装 clang-format）。与功能测试解耦，格式问题不阻塞代码验证。

**覆盖率门禁**：在 `nightly.yml` 的 coverage job 里，从 `summary.txt` 提取 Branch 覆盖率数字，与阈值比较（如 `< 60% 则失败`）。注意覆盖率下降常因新增未测代码而非删除测试，阈值门禁要谨慎。

## 故障排查

**guard job 失败**：通常是某个 `check_*.cmake` 守护脚本发现架构违规（如 include 方向、命名规约、公共 API 边界）。看失败信息里的 `VIOLATIONS:` 清单，对应修源码。这类失败最快修——纯文本规则，无需理解编译。

**Conan 缓存导致依赖旧**：改了 `conanfile.py` 后 CI 应自动重建缓存（key 含文件哈希）。若仍用旧包，检查 `CONAN_CACHE_KEY` 是否正确更新。

**JSBSim 数据 clone 失败**（flight-dynamic job）：网络问题或上游仓库变动。`git clone --depth 1` 应该很稳定；若失败重试即可。注意 JSBSim 源码库较大，浅克隆仍需数十秒。

**Coverage 报告为空**：确认 `llvm-ninja-coverage` preset 确实启用了 `ENABLE_COVERAGE`（configure 输出应含 `Coverage: Enabled`），且 `llvm-profdata`/`llvm-cov` 可用（`macos-14` runner 自带，via `xcrun`）。
