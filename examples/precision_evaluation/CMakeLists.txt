# examples/precision_evaluation/CMakeLists.txt
# 精度评估层独立示例（需求 3.2.1.6.3）：双星 SBIRS + 融合 + 推演的真值对照
# 五项误差（角度/双星交会/速度/落点/发射点）+ AHP 综合评分。评估会话内部自持
# 双星 + 融合 + 推演整链（评估层编排，非传感器组件模型），故为独立可执行而非
# component_attachment 组件。场景硬编码（复刻单测配方），无 JSON/日志设施依赖。

add_executable(precision_evaluation_demo precision_evaluation_demo.cpp)
target_link_libraries(precision_evaluation_demo PRIVATE ${PROJECT_ALIAS})

# 手动运行：./build/<preset>/bin/precision_evaluation_demo [--cycles <n>]
# 验收日志流：配置加 -DONEQ_ENABLE_PRECISION_EVALUATION_LOG=ON 后，库内
# [PrecisionEval] 事件流经 spdlog 默认 logger 输出（本示例不装文件 sink）。
