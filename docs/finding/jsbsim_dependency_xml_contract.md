# JSBSim 依赖与 XML 数据合同基线

> 日期：2026-05-29
> 目的：冻结 flight_dynamic 重构前的 JSBSim binary、XML 数据、项目 XML patch、测试 baseline 和已知副作用。

## 1. 当前 baseline

### 父仓库

| 项 | 当前值 |
|----|--------|
| 工作目录 | `/Users/aurora/Code/1q` |
| 分支 | `main` |
| 提交 | `326e54c` |
| JSBSim Conan 依赖 | `jsbsim/1.3.1` |
| release-local 测试配置 | `ENABLE_TESTING=ON` |

### flight_dynamic baseline 命令

```bash
cmake --build --preset llvm-ninja-release-local --target 1q_unit_tests
build/llvm-ninja-release-local/bin/1q_unit_tests \
  --gtest_filter='FlightDynamicTest.*:FlightDynamicRobustnessTest.*:*AircraftManeuverTest*:FdAircraftProbe.*'
```

### flight_dynamic baseline 结果

| 项 | 数量 |
|----|------|
| 总用例 | 200 |
| 通过 | 192 |
| 跳过 | 6 |
| 失败 | 2 |

失败项：

| 测试 | 机型 | 现象 |
|------|------|------|
| `FighterModels/AircraftManeuverTest.FlyToWaypoint/5` | `f22` | `DoTrim(0)` 异常后继续运行；最终距离 `4215.24m`，未达到 `init_dist * 0.5`，状态仍为 executing |
| `TransportModels/AircraftManeuverTest.Orbit/7` | `Concorde` | 最终 `altitude_geod_m = -0.90644`，判定坠毁 |

默认跳过探针：

| 测试 | 启用变量 |
|------|----------|
| `FdAircraftProbe.EmitsAircraftProfileCsv` | `FD_RUN_AIRCRAFT_PROBE=1` |
| `FdAircraftProbe.EmitsWaypointSweepCsv` | `FD_RUN_WAYPOINT_PROBE=1` |
| `FdAircraftProbe.EmitsOrbitSweepCsv` | `FD_RUN_ORBIT_PROBE=1` |
| `FdAircraftProbe.EmitsF22FocusedCsv` | `FD_RUN_F22_PROBE=1` |
| `FdAircraftProbe.EmitsF22ThrottleCsv` | `FD_RUN_F22_THROTTLE_PROBE=1` |
| `FdAircraftProbe.EmitsF22EnvelopeCsv` | `FD_RUN_F22_ENVELOPE_PROBE=1` |

## 2. 当前依赖关系

### Conan binary

`conanfile.py` 中非 Windows 依赖为：

```python
_JSBSIM_DEPS_NON_WINDOWS = {
    "jsbsim": "jsbsim/1.3.1",
}
```

`cmake/ProjectDependencies.cmake` 的 Conan 模式：

```cmake
find_package(jsbsim CONFIG REQUIRED)
add_library(JSBSim::JSBSim ALIAS jsbsim::jsbsim)
```

含义：macOS/Linux 的 Conan 构建链接 ConanCenter 提供的 JSBSim binary。

当前 CMake configure 输出会显式记录两条来源：

```text
JSBSim binary source: conan:jsbsim/1.3.1
JSBSim data source: vendor:third_party/jsbsim (/Users/aurora/Code/1q/third_party/jsbsim)
```

这代表当前默认模式是：**C++ binary 来自 Conan，运行 XML 数据来自 vendored `third_party/jsbsim`。**

### vendored source/data

`third_party/jsbsim` 是独立嵌套 Git 工作树，不是父仓库 tracked 文件，也不是父仓库 submodule 条目。

当前嵌套仓库状态：

| 项 | 当前值 |
|----|--------|
| remote | `https://github.com/JSBSim-Team/jsbsim.git` |
| HEAD | `3b25f25e` |
| tag | `v1.3.1` |
| describe | `v1.3.1-dirty` |
| CMake version | `1.3.1` |
| config version | `2.0` |

父仓库 `git status` 不会展示 `third_party/jsbsim` 内部 XML/source dirty 状态。因此，仅凭父仓库 commit 不能完整复现当前 JSBSim XML 数据。

## 3. 当前 XML patch 状态

### 项目注入的系统

当前这些 aircraft XML 含有项目相关的 `Navigation`、`GNCUtilities` 或 `Autopilot` 注入：

```text
737, A4, B17, B747, Boeing314, C130, Concorde, DHC6, F4N, F80C,
L410, MD11, OV10, c172p, c172r, c172x, c182, c310, f15, f16, f22,
global5000
```

该列表来自：

```bash
rg -l '<system file="GNCUtilities"|<system file="Autopilot"|<system name="Navigation"' \
  third_party/jsbsim/aircraft
```

注意：`global5000` 当前也包含这些系统引用，但不在现有 `AircraftManeuverTest` 默认机型集合内。后续合同提取需要区分“已注入 XML”与“被回归测试覆盖”。

### 当前嵌套仓库 dirty 范围

`git -C third_party/jsbsim diff --stat` 显示：

| 类型 | 规模 |
|------|------|
| aircraft XML 注入 | 多个已测试机型每个约 10-11 行新增 |
| upstream aircraft 删除 | 大量未使用 aircraft 目录被删除 |
| 总 diff | 122 files changed, 191 insertions, 33378 deletions |

这说明当前 `third_party/jsbsim` 不是干净 upstream `v1.3.1` 数据集，而是裁剪并注入过的本地工作树。

## 4. 输出副作用

常规测试运行期间仍出现以下 JSBSim 输出副作用：

| 机型/文件 | 现象 |
|-----------|------|
| `B17.xml` | 多次尝试打开 `JSBoutB17.csv`，测试输出包含 `ERROR: unable to open the file JSBoutB17.csv` |
| `c172x.xml` | aircraft XML 包含 `JSBout172B.csv` 和 socket output |
| `c310.xml` | aircraft XML 包含 `C310.csv` 和 socket output |
| `f16.xml` | aircraft XML 中有 `f16_datalog.csv` output 文本，但当前在注释块内 |
| `f22.xml` | aircraft XML 中有 `f16_datalog.csv` output 文本，但当前在注释块内 |
| `F80C.xml` | aircraft XML 包含 `F80.csv` |

当前父仓库根目录已有未跟踪输出：

```text
JSBout172B.csv
JSBoutB17.csv
docs/maneuvers/*/*.png
```

这些文件不应作为阶段 1 的修复对象直接删除；它们是当前副作用基线。后续应通过输出路径、禁用 XML output 或测试夹具清理来解决。

### 输出规则

后续代码和 XML 变更必须遵守以下规则：

1. 常规单元测试不得向仓库根目录写入 JSBSim XML output。
2. CSV、TABULAR、SOCKET、FLIGHTGEAR output 只能在显式诊断模式中启用。
3. 诊断输出应重定向到 build tree 或测试临时目录，不能落在 source tree 根目录。
4. 如果短期不能禁用模型自带 output，测试夹具必须在运行前后隔离或清理相关文件。
5. socket/FlightGear output 在常规测试中默认关闭；启用时必须使用显式环境变量或诊断配置。
6. 在确定 overlay/patch 流程前，不直接为了测试副作用修改 upstream aircraft XML；先在 adapter/test fixture 层处理。

## 5. 当前长期策略建议

推荐长期策略：**将 `third_party/jsbsim` 作为 flight_dynamic 的权威 JSBSim source+data 基线，并把它从隐式嵌套 dirty 工作树提升为可追溯依赖单元。**

理由：

1. 项目已经修改 aircraft XML，并依赖 `third_party/jsbsim/aircraft|engine|systems` 作为运行数据根。
2. Windows/VS2015 生产路径已经从 `third_party/jsbsim` 源码构建。
3. JSBSim XML 与 C++ 执行器是耦合模型；Conan binary + 未版本化 XML dirty tree 不利于复现。
4. 如果继续使用 Conan binary，必须保证 binary 版本、XML tag 和 patch 集三者在文档和构建配置中显式绑定。

短期执行策略：

- 保持现有 Conan binary 链接不变，避免在阶段 1 引入大范围构建风险。
- 先记录 `third_party/jsbsim` 的 upstream tag、dirty diff 和项目注入列表。
- 后续阶段再选择以下二选一：
  - 方案 A：把 vendored JSBSim 作为正式 submodule 或 subtree，并提交项目 patch。
  - 方案 B：保留 Conan binary，但把 XML 数据拆成可版本化 aircraft-data 包，并提供 patch 生成流程。

## 6. 阶段 1 验收状态

| 条目 | 状态 |
|------|------|
| 梳理 Conan `jsbsim/1.3.1` | 完成 |
| 梳理 `third_party/jsbsim` 来源 | 完成 |
| 记录 XML patch 列表 | 完成阶段 1 基线；property 合同见阶段 2 文档 |
| 建立版本合同文档 | 完成 |
| 选择长期策略 | 已给出建议，尚未改构建 |
| CMake/config 明确模式名 | 完成：configure 输出 binary/data source |
| XML output 副作用规则 | 完成：已制定测试/诊断输出规则，修复留到阶段 4/8 |

## 7. 后续动作

1. 为 `third_party/jsbsim` 增加可追溯元数据文件或父仓库文档入口。
2. 将阶段 2 控制合同代码化为 `AircraftControlProfile` 的显式字段。
3. 阶段 3 用裸机基线矩阵区分初态、trim、FCS 和任务几何问题。
