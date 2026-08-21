# Windows 本机构建注记（v141）

Status: active
Last-reviewed: 2026-08-21
Authority: local build environment, delivery tier runbook

CLAUDE.md 只保留命令流；本文件承载 Windows 本机开发的环境说明、排障顺序、
VS2015 交付档手册与历史事件。会话门禁的权威简版见
`.cursor/rules/windows-git-bash-build.mdc`。

## 平台差异总览（macOS 主线 vs Windows v141）

macOS 是开发/CI 主线（完整 Conan 依赖集）；本机 Windows 使用 legacy v141 工具集
（裁剪依赖 + 内置文件日志）。Preset、生成器与依赖集差异：

| Aspect | macOS (mainline) | Windows (local v141) |
|---|---|---|
| preset | `llvm-ninja-debug(-local)` / `llvm-ninja-release(-local)` / `llvm-ninja-coverage` | `VisualStudio.15.0-amd64` |
| Generator | Ninja, single-config（build_type 随 preset 固定） | VS2026 generator + v141 (14.16) toolset, multi-config（`--config Debug/Release` 选档） |
| conan install | 每 preset 一次，单一 build_type | bootstrap 一次装 Debug + Release |
| Logging | spdlog/fmt (Conan) | 无第三方 logger；内置 `ProjectFileLog`（`ONEQ_ENABLE_FILE_LOG` 门控，写 `1q_library.log`）；`component_attachment` 示例层自带 std::ofstream 后端（`CA_LOG_BACKEND_SPDLOG=0`） |
| HighFive / JSBSim | Conan 预装 | 均未装；FD=ON 时 JSBSim 需 third_party 源码或预编译树 |
| Coverage | `llvm-ninja-coverage` | 无 |

日常开发用 `*-local` 变体（机器本地 `CMakeUserPresets.json`，不进 git）；CI 用
`CMakePresets.json` 基础 preset。Trap：repo preset `llvm-ninja-release` 的
binaryDir 是 `build/llvm-ninja-release`（无 `-local` 后缀的历史遗留），而
`llvm-ninja-debug` 指向 `build/llvm-ninja-debug-local`——安装路径以实际 binaryDir 为准。

## 一次性环境准备

1. **Git Bash**（本机 Windows 开发唯一合法 shell；WSL/system32 bash 不注入 PATH 且
   解析不了 `ctest.exe`）。Cursor/VS Code 打开本仓库时默认终端已由仓库提交的
   `.vscode/settings.json` 锁定为 Git Bash。
2. `source scripts/activate_1q_git_bash.sh`（建议写入 `~/.bashrc` 持久化；activate
   会补 cmake/ctest PATH、自愈 `core.hooksPath=.githooks`）。机器本地 cmake 路径
   覆盖：`scripts/1q_env.local.sh`（模板见 `.example`，已 gitignore）。
3. 建议把仓库目录加入 Windows Defender 排除（管理员 PowerShell：
   `Add-MpPreference -ExclusionPath 'D:\1q\1q'`），减少 `.obj` 文件锁并提速构建。

自检：`scripts/1q.sh doctor`——`cmake=`/`ctest=` 有路径、`uname` 为 MINGW*/MSYS*、
无 `SHELL_WARNING`。

## 排障顺序（避免打转）

按序归因，**禁止跳过前几步去改业务代码碰运气**：

1. **壳/PATH**：`uname -s` 须 MINGW*/MSYS*。若是 `Linux` + `/mnt/d/...` 路径 →
   在 WSL 里，换 Git Bash。错误壳的典型症状：“cmake 能跑、ctest NOT FOUND、
   构建测试反复空转”。
2. **doctor**：见上；失败先修壳/PATH。
3. **BOM**：带中文注释的 `.h/.cpp` 缺 UTF-8 BOM → C4819/C1083 类问题。
   pre-commit 钩子会在提交时自动补齐；**禁止**删中文注释来消 C4819。
4. **陈旧产物/文件锁**：见下节。
5. 最后才是编译错误本身。

## 陈旧增量产物 / 文件锁

- 症状：public 头布局变更后，模块测试批量 SEH `0xc0000005` / 无关 `bad_alloc`。
  原因：VS 生成器 `.tlog` 增量依赖跟踪对头文件布局移动不敏感，未全量重编，
  Debug 加固检查放大为崩溃。处理：`scripts/1q.sh clean-stale VisualStudio.15.0-amd64 <module_slug>`
  后重建（先于逻辑排查执行）。
- 历史教训（2026-08-17 erratum，2026-08-21 更正）：`SbirsExclusionCauseRecorderTest`
  整批 bad_alloc 曾被记为"已知基线失败"，实为上述陈旧产物；删除
  `build/.../src/sbirs_sensor` 产物强制全量重编后 203 个 sbirs 单测全绿。
  早期记录将其归因于 "Unity builds" 是错的——Unity 构建助手存在于
  `cmake/features/UnityBuild.cmake` 但从未启用。
- 写 `.obj` 报 `Permission denied`：先解除 IDE/杀毒占用（Defender 排除见上），
  不要当逻辑 bug。
- `integration::airborne_radar` 0xc0000409 是独立的既有问题，与本节无关。

## VS2015 交付档（客户集成）

**存在原因**：客户的 VS2015 (v140, any Update) 环境在自己 TU 里编译我们的头文件，
无法要求加编译选项。无 BOM 的 UTF-8 会被按系统代码页（GBK）误读，中文注释错乱。
因此交付 = C++11 + 全部受控 C/C++ 文件带 UTF-8 BOM + 全链**无 `/utf-8`**（含
INTERFACE 传播）。BOM 使 cl.exe 无条件按 UTF-8 解码。维护链：
`scripts/utf8_bom.py`（convert/check/strip）+ pre-commit 自动补齐 + CI `check`
守卫 + `.editorconfig` 声明 `utf-8-bom`。

**Preset 选择**：

| Preset | 用途 |
|---|---|
| `1q_log_vs2015` | **交付验证**：v140 x64，无 Conan（先跑 `fetch_third_party.bat`），C++11，SBIRS/RIR 验收日志 ON |
| `VisualStudio.14.0-amd64-none` | 交付基座（同上去掉验收日志） |
| `VisualStudio.15.0-amd64` | Windows 开发主线：VS2026 + v141，C++17，Conan |
| `llvm-ninja-*` | macOS CI/开发主线 |
| `VisualStudio.14.0-amd64` | VS2015 + Conan——本机不可用（CMake 4.3.1 vs v140 toolset） |

**交付验证流程**（端到端约 15 分钟）：

1. `cmake --preset 1q_log_vs2015 && cmake --build --preset 1q_log_vs2015-release`
2. `cmake --install build/1q_log_vs2015 --config Release`
3. 同步 install → consumer（`D:\1q\1q_consumer`）：`include/1q/*`、`lib/1q.lib`、
   仓库 `examples/` → consumer `src/`（BOM 零改动镜像）。
4. consumer 以 **C++11、无 `/utf-8`** 构建（模拟客户 vcxproj）：
   `cmake -G "Visual Studio 14 2015" -A x64 && cmake --build --config Release`
5. 跑 demo 3 周期；构建日志零 C4819；`1q_library.log` 有 `[SbirsAccept]`/`[RirAccept]`。

## 历史事件：UCRT 注册表死路径（已修复 2026-08-21）

独立安装的 x64 调试工具（WinDbg）曾把 64 位注册表视图
`HKLM\SOFTWARE\Microsoft\Windows Kits\Installed Roots\KitsRoot10` 覆盖为仅含
Debuggers/Catalogs 的目录，v141 的 ucrt.props 解析到死路径——裸目录构建报
`corecrt.h` not found (C1083) / LNK1104 (ucrtd.lib)（2026-08-19 复现）。当时的
规避是 preset/脚本注入 `UCRTContentRoot` + 禁止裸目录构建 + `scripts/bin/cmake`
拦截。2026-08-21 机器级修复：两个注册表视图均指向真实 SDK
（`C:\Program Files (x86)\Windows Kits\10\`），全部规避已退役（preset 不再注入
环境变量、拦截已移除），裸目录构建与 IDE MSBuild 恢复可用。

复发识别：裸构建再次批量 C1083/LNK1104（多为 SDK/调试器重装改写注册表）→
重新修注册表值，**不要**恢复环境变量管道。
