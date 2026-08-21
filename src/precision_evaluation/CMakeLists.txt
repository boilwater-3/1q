# precision_evaluation 模块源文件清单（评估层：真值对照误差提取 + AHP 指标体系）。

oneq_add_component(precision_evaluation_core SOURCES
    AhpEvaluator.cpp
    DualLosFix.cpp
    PrecisionEvaluationMetrics.cpp
    PrecisionEvaluationSession.cpp
    PrecisionAcceptanceRecords.cpp
    SbirsBearingAdapter.cpp
)
# 精度评估验收日志（[PrecisionEval] 事件流）：仅本模块内部消费
# （PrecisionEvaluationLog.h）。
if(ONEQ_ENABLE_PRECISION_EVALUATION_LOG)
    target_compile_definitions(precision_evaluation_core PRIVATE
        ONEQ_ENABLE_PRECISION_EVALUATION_LOG=1)
endif()
