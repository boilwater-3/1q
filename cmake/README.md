# CMake 配置模块

本目录包含项目的 CMake 配置模块，采用模块化设计便于维护和复用。

## 模块列表

### 1. ProjectOptions.cmake

**用途**: 集中定义所有用户可配置选项  
**包含选项**:

- `BUILD_SHARED_LIBS` - 构建动态库/静态库
- `ENABLE_TESTING` - 启用测试支持
- `ENABLE_EXAMPLES` - 构建示例程序
- `ENABLE_INSTALL` - 启用安装规则
- `ENABLE_UNITY_BUILD` - Unity Build 加速编译
- `USE_CCACHE` - ccache 缓存加速
- `ENABLE_PCH` - 预编译头
- `ENABLE_WARNINGS` - 额外警告
- `ENABLE_CLANG_TIDY` - 静态分析
- `STACK_SIZE_OPTION` - 栈大小配置
- `PACKAGE_MANAGER` - 包管理器选择（none/vcpkg/conan）

### 2. FeatureUnityBuild.cmake

**用途**: 配置 Unity Build 特性  
**功能**:

- 根据 `ENABLE_UNITY_BUILD` 选项启用/禁用
- 配置批次大小（默认 16 个文件/批次）
- MSVC 自动添加 `/bigobj` 标志

### 3. FeatureCCache.cmake

**用途**: 配置 ccache 编译缓存  
**功能**:

- 自动搜索 ccache 可执行文件
- 配置 C/C++ 编译器启动器
- 显示版本信息和友好错误提示

### 4. FeaturePrecompiledHeaders.cmake

**用途**: 预编译头配置通知  
**说明**: 实际的 PCH 应在各个目标中单独配置

### 5. FeatureClangTidy.cmake

**用途**: 配置 clang-tidy 静态分析  
**功能**:

- 多版本搜索（clang-tidy-18/17/16）
- 自动检测 `.clang-tidy` 配置文件
- 提供默认检查项
- 显示安装说明

### 6. CompilerMSVC.cmake

**用途**: MSVC 编译器特定配置  
**包含**:

- 栈大小配置（通过 `/STACK` 标志）
- 19 个额外警告标志
- Debug/Release/RelWithDebInfo/MinSizeRel 优化配置
- LTCG（链接时代码生成）
- 函数级链接优化

### 7. CompilerGNU.cmake

**用途**: GCC/Clang 编译器特定配置  
**包含**:

- 栈大小配置（通过 `-Wl,-z,stack-size`）
- 13 个通用 + 6 个 GCC 特定警告标志
- LTO（链接时优化）
- 未使用节移除（`--gc-sections`）
- 位置无关代码（PIC）

### 8. PackageManagerVcpkg.cmake

**用途**: vcpkg 包管理器依赖配置  
**功能**:

- 检查 CMAKE_TOOLCHAIN_FILE 配置
- 集中管理 find_package 依赖声明
- 自动链接到主库目标
- Windows DLL 运行时安装支持

### 9. PackageManagerConan.cmake

**用途**: Conan 包管理器依赖配置  
**功能**:

- 检查 Conan 工具链文件
- 支持 Conan 2.x CMakeToolchain
- 集中管理依赖声明
- 提供 DLL 安装指导

## 模块依赖关系

```
CMakeLists.txt (主文件)
  ├── ProjectOptions.cmake         # 必须最先包含
  ├── FeatureUnityBuild.cmake      # 依赖 ENABLE_UNITY_BUILD
  ├── FeatureCCache.cmake          # 依赖 USE_CCACHE
  ├── FeaturePrecompiledHeaders.cmake  # 依赖 ENABLE_PCH
  ├── FeatureClangTidy.cmake       # 依赖 ENABLE_CLANG_TIDY
  ├── Compiler{MSVC|GNU}.cmake     # 根据编译器选择一个
  └── PackageManager{Vcpkg|Conan}.cmake  # 根据 PACKAGE_MANAGER 选择（可选）

src/CMakeLists.txt (在此之后)
  └── 包管理器模块会链接依赖到 ${PROJECT_CORE_TARGET}
```

## 使用方法

### 在主 CMakeLists.txt 中包含

```cmake
# 包含配置模块
include(cmake/ProjectOptions.cmake)
include(cmake/FeatureUnityBuild.cmake)
include(cmake/FeatureCCache.cmake)
include(cmake/FeaturePrecompiledHeaders.cmake)
include(cmake/FeatureClangTidy.cmake)

# 编译器特定配置
if(MSVC)
    include(cmake/CompilerMSVC.cmake)
else()
    include(cmake/CompilerGNU.cmake)
endif()

# 包管理器配置（在 src/ 目标创建之后）
if(PACKAGE_MANAGER STREQUAL "vcpkg")
    include(cmake/PackageManagerVcpkg.cmake)
elseif(PACKAGE_MANAGER STREQUAL "conan")
    include(cmake/PackageManagerConan.cmake)
endif()
```

### 配置选项示例

```bash
# 启用 Unity Build 和 ccache
cmake -B build -DENABLE_UNITY_BUILD=ON -DUSE_CCACHE=ON

# 启用静态分析（慎用，会显著增加编译时间）
cmake -B build -DENABLE_CLANG_TIDY=ON

# 配置大栈空间
cmake -B build -DSTACK_SIZE_OPTION=LARGE_PROJECT

# 使用 vcpkg 包管理器
cmake -B build -DPACKAGE_MANAGER=vcpkg \
  -DCMAKE_TOOLCHAIN_FILE=<vcpkg_root>/scripts/buildsystems/vcpkg.cmake

# 使用 Conan 包管理器
cd build
conan install .. --output-folder=. --build=missing
cmake .. -DPACKAGE_MANAGER=conan
```

### 配置选项示例

```bash
# 启用 Unity Build 和 ccache
cmake -B build -DENABLE_UNITY_BUILD=ON -DUSE_CCACHE=ON

# 启用静态分析（慎用，会显著增加编译时间）
cmake -B build -DENABLE_CLANG_TIDY=ON

# 配置大栈空间
cmake -B build -DSTACK_SIZE_OPTION=LARGE_PROJECT
```

## 模块化优势

1. **可维护性**: 每个模块职责单一，便于定位和修改
2. **可复用性**: 模块可在多个项目间共享
3. **可测试性**: 独立模块便于单独测试
4. **可读性**: 主 CMakeLists.txt 从 520 行缩减至 ~180 行
5. **版本控制**: 模块化便于跟踪历史变更

## 扩展指南

### 添加新的特性模块

1. 创建 `cmake/FeatureNewFeature.cmake`
2. 添加选项到 `ProjectOptions.cmake`
3. 在主 CMakeLists.txt 中 include
4. 更新本 README

### 添加新的编译器支持

1. 创建 `cmake/CompilerNewCompiler.cmake`
2. 在主 CMakeLists.txt 中添加编译器检测逻辑
3. 更新本 README

## 最佳实践

1. **包含顺序**: 始终先包含 ProjectOptions.cmake
2. **错误处理**: 在模块中使用 `FATAL_ERROR` 提示必需工具缺失
3. **状态消息**: 使用树形结构（└─）提升可读性
4. **注释完整**: 每个模块开头应有用途说明
5. **选项验证**: 在模块中验证选项有效性
