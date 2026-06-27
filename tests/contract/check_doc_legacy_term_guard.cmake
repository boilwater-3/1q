# check_doc_legacy_term_guard.cmake
#
# 守护规范性文档不残留已收口的旧入口术语(HARD 阻断)。
#
# 背景:公开 API 已从独立工厂类(SessionFactory)收口为静态成员
# (Session::Create / CreateWithValidation / CreateWithDecisionEngine),
# 决策 SPI 命名空间从 extension:: 收口为 session::。代码与安装/白名单
# 守护已很强,但规范性文档仍可能漂移回旧术语,误导后续维护者。
#
# 本守护扫描"规范性文档"(docs/common + 五个模块文档),
# 禁止出现以下已收口入口:
#   - RadarSessionFactory::CreateWithDecisionEngine
#   - RadarSessionFactory::Create
#   - extension::ITacticalDecisionEngine  (现为 session::ITacticalDecisionEngine)
#   - EosSessionFactory / SarSessionFactory / EsrSessionFactory  (ghost 类,已删)
#
# 当前 docs 结构只允许 docs/common 与五个模块目录;历史审查快照不再常驻 docs。

cmake_minimum_required(VERSION 3.16)

set(LEGACY_TERMS
    "RadarSessionFactory::CreateWithDecisionEngine"
    "RadarSessionFactory::Create"
    "extension::ITacticalDecisionEngine"
    "EosSessionFactory"
    "SarSessionFactory"
    "EsrSessionFactory"
)

# 规范性文档(显式列举,不扫描历史目录):
#   - docs/common/*.md
#   - docs/<module>/*.md
set(NORMATIVE_DOC_FILES "")
foreach(_doc_dir
        common
        airborne_radar
        electro_optical_sensor
        electronic_surveillance_radar
        flight_dynamic
        sar)
    file(GLOB _docs_in_dir "${SOURCE_DIR}/docs/${_doc_dir}/*.md")
    list(APPEND NORMATIVE_DOC_FILES ${_docs_in_dir})
endforeach()

set(VIOLATIONS "")
foreach(doc_file ${NORMATIVE_DOC_FILES})
    file(STRINGS "${doc_file}" _lines)
    foreach(line ${_lines})
        foreach(term ${LEGACY_TERMS})
            string(FIND "${line}" "${term}" _idx)
            if(NOT _idx EQUAL -1)
                list(APPEND VIOLATIONS "${doc_file}: ${term}\n    > ${line}")
            endif()
        endforeach()
    endforeach()
endforeach()

if(VIOLATIONS)
    set(_err "规范性文档中残留已收口的旧入口术语:\n\n")
    foreach(v ${VIOLATIONS})
        string(APPEND _err "  ${v}\n\n")
    endforeach()
    message(FATAL_ERROR
        "${_err}"
        "这些术语对应的 API 已收口:\n"
        "  - SessionFactory 工厂类 → Session::Create / CreateWithValidation 静态成员\n"
        "  - extension::ITacticalDecisionEngine → session::ITacticalDecisionEngine\n"
        "  - Eos/Sar/Esr SessionFactory → 已删除(ghost 类)\n\n"
        "修复:将规范性文档中的旧入口同步为当前 API。\n"
        "若是历史审查记录,不应常驻当前 docs 结构;请压缩为 history.md 结论或从 git 历史追溯。")
endif()

list(LENGTH VIOLATIONS _violation_count)
message(STATUS "[doc-legacy-term] 通过:规范性文档无旧入口术语残留 (${_violation_count} 处违规)。")
