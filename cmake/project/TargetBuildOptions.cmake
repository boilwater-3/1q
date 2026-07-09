# Applies target-scoped build options to project-owned targets.

set(_oneq_build_option_targets ${PROJECT_CORE_TARGET} ${ONEQ_OBJECT_TARGETS})

oneq_apply_unity_build(TARGETS ${_oneq_build_option_targets})

if(MSVC)
    oneq_apply_msvc_options(
        TARGETS ${_oneq_build_option_targets}
        LINK_TARGETS ${PROJECT_CORE_TARGET}
        ENABLE_WARNINGS ${ENABLE_WARNINGS}
        STACK_SIZE_OPTION ${STACK_SIZE_OPTION})
else()
    oneq_apply_clang_gcc_options(
        TARGETS ${_oneq_build_option_targets}
        LINK_TARGETS ${PROJECT_CORE_TARGET}
        ENABLE_WARNINGS ${ENABLE_WARNINGS}
        STACK_SIZE_OPTION ${STACK_SIZE_OPTION})
endif()

oneq_apply_coverage_options(
    TARGETS ${_oneq_build_option_targets}
    LINK_TARGETS ${PROJECT_CORE_TARGET})

oneq_apply_precompiled_headers(TARGETS ${_oneq_build_option_targets})

unset(_oneq_build_option_targets)
