# tests 目录约定

## 分层规则
- `unit/`: 单元测试（含原压力测试）。
- `integration/`: 跨组件或会话级集成测试。
- `contract/`: 公共 API/头文件稳定性与边界约束测试。
- `consumer/`: 安装后消费者样例与安装验证。

## 归档规则
- 新增测试文件禁止放在 `tests/` 根目录，必须放入上述分层目录。
- 域归属由文件名前缀（`airborne_`、`esr_`、`eos_`）编码，不再使用域子目录。
- 共享测试辅助代码只放 `fixtures/` 或 `mocks/`，避免复制。

## 命名规则
- 所有测试文件统一命名：`{domain}_{descriptive_name}_test.cpp`。
  - `domain` 取 `airborne` \| `esr` \| `eos`，与所在域目录对齐。
  - `contract/` 下的跨域公共 API 测试不带域前缀。
- 文件名中**不**再包含层级后缀（`_unit_`、`_integration_`、`_stress_`），层级由目录路径和 CTest label 编码。
- 示例：`airborne_kalman_filter_test.cpp`、`esr_kdtree_clusterer_test.cpp`、`eos_pipeline_test.cpp`。

## CTest 运行建议
- 全量：`ctest --preset llvm-ninja-debug-local -Q --output-on-failure`
- 仅单元：`ctest --preset llvm-ninja-debug-local -Q --output-on-failure -L unit`
- 仅集成：`ctest --preset llvm-ninja-debug-local -Q --output-on-failure -L integration`
- 仅契约：`ctest --preset llvm-ninja-debug-local -Q --output-on-failure -L contract`
