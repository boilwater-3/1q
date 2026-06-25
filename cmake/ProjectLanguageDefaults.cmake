# Shared language defaults used by CMakeLists and toolchain bootstrap.
# Keep these values as the single source of truth.
#
# 构建标准保持 C++17：非 Windows 平台的 jsbsim/1.3.1 依赖要求 cppstd>=17。
# C++11 兼容性不通过"降低构建标准"实现，而通过"公共头守 C++11 子集"的独立
# 约束实现（见 docs/output_observability_contract.md 与命名 lint）：
# include/1q/ 公共头不得使用 C++14/17 特性（std::optional/variant、if constexpr、
# structured bindings、std::make_unique、auto 返回类型推导等），以保证 VS2015
# 消费方可编译；src/ .cpp 实现可使用 C++17（jsbsim 等依赖需要）。

set(PROJECT_DEFAULT_CXX_STANDARD 17)
