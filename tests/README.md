# tests 目录约定

## 分层规则
- `unit/`: 模块级单元测试。
- `integration/`: 跨组件或会话级集成测试。
- `contract/public_api/`: 公共 API/头文件稳定性与边界约束测试。
- `stress/`: 长耗时/高负载测试（默认可关闭）。
- `fixtures/`: 测试夹具与共享测试基类。
- `mocks/`: Mock 接口与测试替身定义。
- `install_consumer/`: 安装后消费者样例与安装验证。

## 归档规则
- 新增测试文件禁止放在 `tests/` 根目录，必须放入上述分层目录。
- `airborne_radar`、`esr`、`eos` 测试优先落到各自域目录。
- 共享测试辅助代码只放 `fixtures/` 或 `mocks/`，避免复制。
- EOS 单元测试文件统一命名为 `eos_*_unit_test.cpp`。
- 新增单元测试文件命名建议统一为 `*_unit_test.cpp`。

## CTest 运行建议
- 全量：`ctest --preset llvm-ninja-debug-local -Q --output-on-failure`
- 仅单元：`ctest --preset llvm-ninja-debug-local -Q --output-on-failure -L unit`
- 仅集成：`ctest --preset llvm-ninja-debug-local -Q --output-on-failure -L integration`
- 仅契约：`ctest --preset llvm-ninja-debug-local -Q --output-on-failure -L contract`
- 仅压力：`ctest --preset llvm-ninja-debug-local -Q --output-on-failure -L stress`
