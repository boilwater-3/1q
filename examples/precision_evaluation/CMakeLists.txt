# examples/precision_evaluation/CMakeLists.txt
# 精度评估层独立示例（需求 3.2.1.6.3）：双星 SBIRS + 融合 + 推演的真值对照
# 五项误差（角度/双星交会/速度/落点/发射点）+ AHP 综合评分。评估会话内部自持
# 双星 + 融合 + 推演整链（评估层编排，非传感器组件模型），故为独立可执行而非
# component_attachment 组件。几何与周期由 scenes/*.json 驱动。

add_executable(precision_evaluation_demo precision_evaluation_demo.cpp)
target_link_libraries(precision_evaluation_demo PRIVATE ${PROJECT_ALIAS})
target_include_directories(precision_evaluation_demo PRIVATE
    "${ONEQ_EXAMPLE_COMMON_DIR}")
target_sources(precision_evaluation_demo PRIVATE ${ONEQ_EXAMPLE_COMMON_SOURCES})
target_compile_definitions(precision_evaluation_demo PRIVATE
    PE_DEFAULT_SCENE_FILE="${CMAKE_SOURCE_DIR}/examples/precision_evaluation/scenes/sbirs_dual_sat_fix/sbirs_dual_sat_fix.json"
    PE_DEFAULT_OUTPUT_DIR="${CMAKE_SOURCE_DIR}/examples/precision_evaluation/log"
    PE_ACCEPTANCE_LOG_ENABLED=$<BOOL:${ONEQ_ENABLE_PRECISION_EVALUATION_LOG}>)

if(NOT ONEQ_ENABLE_PRECISION_EVALUATION_LOG)
    message(STATUS "precision_evaluation: ONEQ_ENABLE_PRECISION_EVALUATION_LOG=OFF -> no "
        "precision_acceptance.log; reconfigure with -DONEQ_ENABLE_PRECISION_EVALUATION_LOG=ON")
endif()
if(NOT ONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG OR NOT ONEQ_ENABLE_FUSION_ACCEPTANCE_LOG
    OR NOT ONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG)
    message(STATUS "precision_evaluation: also enable "
        "ONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG / FUSION / INFERENCE for "
        "sbirs/fusion/inference_acceptance.log under the scene output dir")
endif()

# 手动运行：
#   ./build/<preset>/bin/precision_evaluation_demo
#       [--scene <path>] [--cycles <n>] [--output-dir <dir>]
# 验收文件：configure 打开 PE + SBIRS + FUSION + INFERENCE 四个开关后，
# demo 把四份验收文件钉到 --output-dir（默认 log/<场景名>/）。
