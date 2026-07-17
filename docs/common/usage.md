# 1q 库消费指南

Status: active

外部项目使用 1q 有两种方式：**直接使用预编译安装树**（零依赖）或 **通过 Conan 管理依赖**（项目标准方式）。

---

## 方式一：直接使用安装树（零额外依赖）

1q 安装后，所有第三方依赖的 CMake 配置也一并部署在 `lib/cmake/1q/deps/` 目录下。下游只需指向安装路径即可，无需单独安装 Eigen3、Boost 等包。

```cmake
# 最小示例
cmake_minimum_required(VERSION 3.16)
project(my_app LANGUAGES CXX)

# 直接指向 1q 的安装路径
find_package(1q REQUIRED CONFIG
    PATHS "D:/path/to/1q/install/lib/cmake/1q"
    NO_DEFAULT_PATH
)

# 链接 1q（第三方依赖自动传递）
add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE 1q::1q)
```

`1qConfig.cmake` 会自动:

1. 将 `deps/` 加入 `CMAKE_PREFIX_PATH`
2. 加载 `Eigen3Config.cmake`、`BoostConfig.cmake`、`nanoflannConfig.cmake`、`flatbuffersConfig.cmake`、`ZLIBConfig.cmake`（均从安装目录解析）
3. 定义 `1q::1q` 静态库 target，携带编译选项、头文件路径、以及传递的第三方依赖

**前提**：1q 的 conan cache 在目标机器上存在（安装树中的 cmake config 指向 conan cache 中的头文件路径）。同一台机器或同一 conan 环境中构建/安装/使用均无问题。

**生产/开发环境典型流程**：

```bash
# 1. 构建并安装 1q（生产机器上仅需执行一次）
scripts/bootstrap_conan.sh VisualStudio.14.0-amd64
cmake --preset VisualStudio.14.0-amd64
cmake --build --preset VisualStudio.14.0-amd64-release
cmake --install build/VisualStudio.14.0-amd64 --config Release

# 2. 下游项目使用（指向安装路径即可）
cmake -S . -B build -D1q_DIR="D:/path/to/install/lib/cmake/1q"
cmake --build build
```

---

## 方式二：通过 Conan 管理依赖（推荐，macOS/CI 标准做法）

外部项目自身也通过 Conan 管理依赖，和 1q 使用相同的 conan profile。1q 的第三方依赖链由 Conan 统一协调。

```python
# conanfile.py（下游项目）
from conan import ConanFile

class MyAppConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    requires = "1q/0.1"

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()
        deps = CMakeDeps(self)
        deps.generate()
```

```cmake
# CMakeLists.txt（下游项目）
cmake_minimum_required(VERSION 3.16)
project(my_app LANGUAGES CXX)

# Conan 已在 CMakePresets.json / CMakeUserPresets.json 中设置好 toolchain
# 或直接用：
find_package(1q REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE 1q::1q)
```

```bash
# 下游构建步骤
conan install . --output-folder=build --build=missing
cmake --preset conan-default
cmake --build build
```

---

## 1q::1q target 属性

安装后检查 `1q::1q` 的关键属性：

| 属性 | 值 |
|---|---|
| 类型 | `STATIC_LIBRARY` |
| 包含目录 | `${INSTALL_PREFIX}/include`（1q public headers） |
| 编译定义 | `ONEQ_STATIC_DEFINE` |
| 编译特性 | `cxx_std_14` |
| 传递链接（`LINK_ONLY`） | `Eigen3::Eigen`、`Boost::boost`、`nanoflann::nanoflann`、`flatbuffers::flatbuffers`、`ZLIB::ZLIB` |

第三方库以 `$<LINK_ONLY:...>` 传递，链接时已编入 `1q.lib`，下游无需关心链接顺序。

---

## 可选模块

构建 1q 时可通过 CMake 选项控制功能范围，影响下游的使用前提：

| 选项 | 默认值 | 说明 | 下游影响 |
|---|---|---|---|
| `ENABLE_TESTING` | OFF | 单元测试 | 下游无需关心 |
| `ENABLE_EXAMPLES` | OFF | 示例程序 | 下游无需关心 |
| `ONEQ_ENABLE_FLIGHT_DYNAMIC` | OFF | 机动飞行模块 | 关闭时 JSBSim 不编译也不导出依赖 |
| `BUILD_SHARED_LIBS` | OFF | 共享/静态库 | 当前仅支持静态库 |
