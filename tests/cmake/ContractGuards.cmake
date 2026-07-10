# Source-level contract guard registration.

function(oneq_add_script_guard test_name script_name)
    add_test(NAME ${test_name}
        COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/${script_name})
endfunction()

oneq_add_script_guard(public_api_boundary_guard check_public_api_boundary.cmake)
oneq_add_script_guard(airborne_include_style_guard check_airborne_include_style.cmake)
oneq_add_script_guard(esr_include_style_guard check_esr_include_style.cmake)
oneq_add_script_guard(eos_include_style_guard check_eos_include_style.cmake)
oneq_add_script_guard(airborne_include_direction_guard check_airborne_include_direction.cmake)
oneq_add_script_guard(esr_include_direction_guard check_esr_include_direction.cmake)
oneq_add_script_guard(eos_include_direction_guard check_eos_include_direction.cmake)
oneq_add_script_guard(public_header_external_dependency_isolation_guard
    check_public_header_external_dependency_isolation.cmake)
oneq_add_script_guard(cross_domain_naming_guard check_cross_domain_naming.cmake)
oneq_add_script_guard(install_manifest_guard check_install_manifest.cmake)
oneq_add_script_guard(doc_legacy_term_guard check_doc_legacy_term_guard.cmake)
oneq_add_script_guard(sar_doc_governance_guard check_sar_doc_governance.cmake)
oneq_add_script_guard(docs_structure_guard check_docs_structure.cmake)
oneq_add_script_guard(cmake_helper_parse_guard check_cmake_helper_parse.cmake)
oneq_add_script_guard(preset_provider_contract_guard check_preset_provider_contract.cmake)
oneq_add_script_guard(cmake_project_layout_guard check_cmake_project_layout.cmake)
oneq_add_script_guard(sar_frozen_sources check_sar_frozen_sources.cmake)

set_tests_properties(
    public_api_boundary_guard
    airborne_include_style_guard
    esr_include_style_guard
    eos_include_style_guard
    airborne_include_direction_guard
    esr_include_direction_guard
    eos_include_direction_guard
    public_header_external_dependency_isolation_guard
    cross_domain_naming_guard
    install_manifest_guard
    doc_legacy_term_guard
    sar_doc_governance_guard
    docs_structure_guard
    cmake_helper_parse_guard
    preset_provider_contract_guard
    cmake_project_layout_guard
    sar_frozen_sources
    PROPERTIES LABELS "contract")
set_tests_properties(public_api_boundary_guard PROPERTIES LABELS "contract;public_api;ci_required")
set_tests_properties(sar_doc_governance_guard PROPERTIES LABELS "contract;sar;ci_required")
set_tests_properties(sar_frozen_sources PROPERTIES LABELS "contract;sar;ci_required")
