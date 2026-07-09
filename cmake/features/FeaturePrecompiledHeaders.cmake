# 预编译头（PCH）配置
# 预编译头可显著加速包含大量第三方库头文件的项目编译

if(ENABLE_PCH)
    message(STATUS "Precompiled Headers: Enabled")
    message(STATUS "  └─ Targets should call target_precompile_headers()")
    # 注意：实际的 PCH 设置应在各个目标的 CMakeLists.txt 中配置
    # 例如：target_precompile_headers(my_target PRIVATE <vector> <string>)
else()
    message(STATUS "Precompiled Headers: Disabled")
endif()
