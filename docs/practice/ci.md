# CI 持续集成

Status: active
Last-reviewed: 2026-07-11
Authority: build & test infrastructure

本仓库的持续集成（CI）由两个 GitHub Actions workflow 组成，按"快速门禁 vs 慢任务"分层。

## Job 矩阵总览

| Workflow | 触发 | Job | 内容 | 阻断 PR |
|---|---|---|---|---|
| `ci.yml` | push / PR | **guard** | `-L contract`：17 个架构守护 + 契约测试，编译前先跑 | ✅ |
| `ci.yml` | push / PR | **build-test** | Debug 库/第一方示例构建 + 安装 consumer + `ci_required` + 全量 `unit` 分区 | ✅ |
| `nightly.yml` | cron 02:00 + 手动 | **performance** | Release 构建 + 性能/cxx11 兼容 gate | ❌ |
| `nightly.yml` | cron + 手动 | **flight-dynamic** | FD=ON + JSBSim 数据 + `unit::flight_dynamic` | ❌ |
| `nightly.yml` | cron + 手动 | **coverage** | 插桩构建 + 覆盖率报告 artifact | ❌ |

设计原则：**CI 一上线就必须是绿的。** `ci_required` 覆盖跨层关键路径与架构守护；完整 `unit` 分区覆盖可重复的局部回归，二者均为 PR 阻断门禁。

## 平台范围

当前仅 macOS（`macos-14` arm64 runner）。理由：
- macOS 路径下 Conan 提供全部依赖（含 JSBSim 预编译包 `jsbsim/1.3.1`），最干净可靠。
- Windows 依赖与安装消费闭包尚未验证，因此项目不再公开 Windows preset；恢复前必须先建立真实的 configure/build/install-consumer 验证。

## 本地复现

CI 跑的每条命令都可在本地复现，便于调试失败：

```bash
# === guard job（contract 守护）===
bash scripts/bootstrap_conan.sh llvm-ninja-debug
cmake --preset llvm-ninja-debug
ctest --test-dir build/llvm-ninja-debug-local -L contract --output-on-failure -j 4

# === build-test job（绿色门禁）===
bash scripts/bootstrap_conan.sh llvm-ninja-debug
cmake --preset llvm-ninja-debug -D ENABLE_EXAMPLES=ON
cmake --build --preset llvm-ninja-debug -j 4
cmake --install build/llvm-ninja-debug-local
cmake -S tests/consumer -B build/consumer/llvm-ninja-debug -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/build/llvm-ninja-debug-local/build/Debug/generators/conan_toolchain.cmake" \
  -DCMAKE_PREFIX_PATH="$PWD/build/install/llvm-ninja-debug"
cmake --build build/consumer/llvm-ninja-debug -j 4
ctest --test-dir build/llvm-ninja-debug-local -L ci_required --output-on-failure -j 4

# === build-test 的 unit 分区 ===
ctest --test-dir build/llvm-ninja-debug-local -L unit --output-on-failure -j 4

# === nightly: performance ===
bash scripts/bootstrap_conan.sh llvm-ninja-release
cmake --preset llvm-ninja-release
cmake --build --preset llvm-ninja-release -j 4
ctest --test-dir build/llvm-ninja-release -L 'performance|sar_performance' --output-on-failure -j 4
ctest --test-dir build/llvm-ninja-release -R sar_cxx11_compat --output-on-failure -j 4

# === nightly: flight-dynamic（需先获取 JSBSim 数据）===
git clone --depth 1 https://github.com/JSBSim-Team/jsbsim.git third_party/jsbsim
bash scripts/bootstrap_conan.sh llvm-ninja-debug
cmake --preset llvm-ninja-debug -D ONEQ_ENABLE_FLIGHT_DYNAMIC=ON
cmake --build --preset llvm-ninja-debug -j 4
ctest --test-dir build/llvm-ninja-debug-local -R '^unit::flight_dynamic$' --output-on-failure -j 4

# === nightly: coverage ===
bash scripts/bootstrap_conan.sh llvm-ninja-coverage
cmake --preset llvm-ninja-coverage
cmake --build --preset llvm-ninja-coverage -j 4
bash tools/coverage_report.sh
```

## 手动触发 nightly

GitHub 仓库 → **Actions** 标签页 → 左侧选 **Nightly** → 右上角 **Run workflow**。

适用于：修复后想立即验证、不想等到凌晨 2 点自动触发。

## 扩展指南

**新增 Windows job**：先新增并验证 Windows Conan preset，再在 `nightly.yml` 加 `runs-on: windows-latest` job。验收必须覆盖 JSBSim / HighFive 的依赖来源、configure、build 和安装后的 consumer；未满足前不要在文档或 presets 中声明 Windows 支持。

**新增 lint job**（clang-format/tidy）：在 `ci.yml` 加独立 job 跑 `cmake --build --target format-check`（需 runner 装 clang-format）。与功能测试解耦，格式问题不阻塞代码验证。

**覆盖率门禁**：在 `nightly.yml` 的 coverage job 里，从 `summary.txt` 提取 Branch 覆盖率数字，与阈值比较（如 `< 60% 则失败`）。注意覆盖率下降常因新增未测代码而非删除测试，阈值门禁要谨慎。

## 故障排查

**guard job 失败**：通常是某个 `check_*.cmake` 守护脚本发现架构违规（如 include 方向、命名规约、公共 API 边界）。看失败信息里的 `VIOLATIONS:` 清单，对应修源码。这类失败最快修——纯文本规则，无需理解编译。

**Conan 缓存导致依赖旧**：改了 `conanfile.py` 后 CI 应自动重建缓存（key 含文件哈希）。若仍用旧包，检查 `CONAN_CACHE_KEY` 是否正确更新。

**JSBSim 数据 clone 失败**（flight-dynamic job）：网络问题或上游仓库变动。`git clone --depth 1` 应该很稳定；若失败重试即可。注意 JSBSim 源码库较大，浅克隆仍需数十秒。

**Coverage 报告为空**：确认 `llvm-ninja-coverage` preset 确实启用了 `ENABLE_COVERAGE`（configure 输出应含 `Coverage: Enabled`），且 `llvm-profdata`/`llvm-cov` 可用（`macos-14` runner 自带，via `xcrun`）。
