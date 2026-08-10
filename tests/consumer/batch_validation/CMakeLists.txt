# 批量场景验证框架（tests/consumer 分区，树内注册）：
# 多场景参数扫描 + CSV 指标 + trace 录制/回放回归。每个模块一个独立
# 可执行程序，证明模块在多场景下的泛用性（非 GoogleTest 业务场景入口，
# 见 tests/README.md「唯一的非 GoogleTest 业务场景入口」注记）。
#
# 本框架只使用公开 Session / Adapter / Replay 接口，并按模块使用公开
# config_loader 或 POD / builder 构造配置。为满足这些外部可观察契约而
# 需要的运行时语义修复仍由对应生产模块和回归测试拥有，不在本目录复制
# 实现（详见 README.md「与现有测试的关系」）。
#
# 依赖说明：各模块的 config_loader.h 经 examples/common/config_loaders/<域>/
# 注入，配置 JSON 经 BATCH_CONFIG_DIR 指向 examples/configs/——本框架
# 单向依赖 examples 层共享便利层（消费方参考的一部分，不属于库 public
# surface；tests/unit/examples 分区同样直接编译 examples 源码，先例一致）。

# 空军雷达批量验证：引用本目录工具头 + airborne_radar 的 config_loader.h。
add_executable(ar_batch_validation ar_batch_validation.cpp)
target_link_libraries(ar_batch_validation PRIVATE ${PROJECT_ALIAS})
target_include_directories(ar_batch_validation PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/examples/common/config_loaders/airborne_radar"
    "${CMAKE_SOURCE_DIR}/examples/common")
target_compile_definitions(ar_batch_validation PRIVATE
    BATCH_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/configs")
target_sources(ar_batch_validation PRIVATE
    "${CMAKE_SOURCE_DIR}/examples/common/json_reader.cpp")

# 光电传感器批量验证：引用本目录工具头 + electro_optical 的 config_loader.h。
add_executable(eos_batch_validation eos_batch_validation.cpp)
target_link_libraries(eos_batch_validation PRIVATE ${PROJECT_ALIAS})
target_include_directories(eos_batch_validation PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/examples/common/config_loaders/electro_optical"
    "${CMAKE_SOURCE_DIR}/examples/common")
target_compile_definitions(eos_batch_validation PRIVATE
    BATCH_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/configs")
target_sources(eos_batch_validation PRIVATE
    "${CMAKE_SOURCE_DIR}/examples/common/json_reader.cpp")

# 电子侦察雷达批量验证：引用本目录工具头 + electronic_warfare 的 config_loader.h。
add_executable(esr_batch_validation esr_batch_validation.cpp)
target_link_libraries(esr_batch_validation PRIVATE ${PROJECT_ALIAS})
target_include_directories(esr_batch_validation PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/examples/common/config_loaders/electronic_warfare"
    "${CMAKE_SOURCE_DIR}/examples/common")
target_compile_definitions(esr_batch_validation PRIVATE
    BATCH_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/configs")
target_sources(esr_batch_validation PRIVATE
    "${CMAKE_SOURCE_DIR}/examples/common/json_reader.cpp")

# SAR 批量验证：引用本目录工具头 + sar 的 config_loader.h
# （共享于 examples/common/config_loaders/sar/，与 AR/ESR/EOS 同源）。
add_executable(sar_batch_validation sar_batch_validation.cpp)
target_link_libraries(sar_batch_validation PRIVATE ${PROJECT_ALIAS})
target_include_directories(sar_batch_validation PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/examples/common/config_loaders/sar"
    "${CMAKE_SOURCE_DIR}/examples/common")
target_compile_definitions(sar_batch_validation PRIVATE
    BATCH_CONFIG_DIR="${CMAKE_SOURCE_DIR}/examples/configs")
target_sources(sar_batch_validation PRIVATE
    "${CMAKE_SOURCE_DIR}/examples/common/json_reader.cpp")

# SBIRS 批量验证：配置与输入均通过公开 POD / builder 构造，无私有 config loader。
# 共享 CSV 工具位于 examples/common/。
add_executable(sbirs_batch_validation sbirs_batch_validation.cpp)
target_link_libraries(sbirs_batch_validation PRIVATE ${PROJECT_ALIAS})
target_include_directories(sbirs_batch_validation PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/examples/common")

if(BUILD_TESTING)
    function(oneq_add_batch_sequence_test domain target)
        add_test(NAME "batch_validation::${domain}"
            COMMAND $<TARGET_FILE:${target}>
                --suite sequence
                --output-dir "${CMAKE_BINARY_DIR}/batch_validation/${domain}")
        set_tests_properties("batch_validation::${domain}" PROPERTIES
            LABELS "batch_validation;${domain}"
            TIMEOUT 120)
    endfunction()

    oneq_add_batch_sequence_test(airborne_radar ar_batch_validation)
    oneq_add_batch_sequence_test(electro_optical_sensor eos_batch_validation)
    oneq_add_batch_sequence_test(electronic_surveillance_radar esr_batch_validation)
    oneq_add_batch_sequence_test(sar sar_batch_validation)
    oneq_add_batch_sequence_test(sbirs_sensor sbirs_batch_validation)
endif()
