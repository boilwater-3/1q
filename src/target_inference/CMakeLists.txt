# target_inference 模块源文件清单。

oneq_add_component(target_inference_core SOURCES
    TargetInferenceEngine.cpp
    InferenceAcceptanceRecords.cpp)
if(ONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG)
    target_compile_definitions(target_inference_core PRIVATE
        ONEQ_ENABLE_INFERENCE_ACCEPTANCE_LOG=1)
endif()
