# EOS 示例构建配置
# 当前目录预留为后续 EOS 示例扩展入口。

find_package(nlohmann_json CONFIG REQUIRED)

set(EOS_EXAMPLE_SOURCES
    eos_quick_start.cpp
    eos_data_collector.cpp
    eos_trace_replayer.cpp
)

foreach(example_src IN LISTS EOS_EXAMPLE_SOURCES)
    get_filename_component(example_name ${example_src} NAME_WE)

    add_executable(${example_name} ${example_src})
    target_link_libraries(${example_name} PRIVATE ${PROJECT_ALIAS} nlohmann_json::nlohmann_json)
    target_include_directories(${example_name} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/examples
    )

    if(ENABLE_INSTALL)
        install(TARGETS ${example_name}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/examples
        )
    endif()

    message(STATUS "  └─ Example(eos): ${example_name}")
endforeach()

list(LENGTH EOS_EXAMPLE_SOURCES eos_example_count)
message(STATUS "Configured ${eos_example_count} EOS example(s)")
