# Compiler/language-standard compatibility probes.

add_test(NAME public_header_cxx11_guard
    COMMAND ${CMAKE_COMMAND}
        -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
        -P ${CMAKE_CURRENT_LIST_DIR}/../compatibility/public_api/check_public_header_cxx11.cmake)
set_tests_properties(public_header_cxx11_guard PROPERTIES LABELS "compatibility;public_api")

get_target_property(_oneq_sar_eigen_include_dirs Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
list(GET _oneq_sar_eigen_include_dirs 0 _oneq_sar_eigen_include_dir)
if(MSVC)
    set(_oneq_sar_cxx11_runner "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/run_sar_cxx11_compat.bat")
    file(GENERATE OUTPUT "${_oneq_sar_cxx11_runner}" CONTENT
"@call \"${CMAKE_GENERATOR_INSTANCE}/VC/Auxiliary/Build/vcvars64.bat\" >nul
@\"${CMAKE_COMMAND}\" -DSOURCE_DIR=\"${CMAKE_SOURCE_DIR}\" -DCXX_COMPILER=\"${CMAKE_CXX_COMPILER}\" -DCXX_COMPILER_ID=MSVC -DEIGEN_INCLUDE_DIR=\"${_oneq_sar_eigen_include_dir}\" -P \"${CMAKE_CURRENT_LIST_DIR}/../compatibility/sar/check_sar_cxx11_compat.cmake\"
")
    add_test(NAME sar_cxx11_compat COMMAND "${_oneq_sar_cxx11_runner}")
else()
    add_test(NAME sar_cxx11_compat
        COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DCXX_COMPILER=${CMAKE_CXX_COMPILER}
            -DCXX_COMPILER_ID=${CMAKE_CXX_COMPILER_ID}
            -DEIGEN_INCLUDE_DIR=${_oneq_sar_eigen_include_dir}
            -P ${CMAKE_CURRENT_LIST_DIR}/../compatibility/sar/check_sar_cxx11_compat.cmake)
endif()
set_tests_properties(sar_cxx11_compat PROPERTIES LABELS "compatibility;sar")
