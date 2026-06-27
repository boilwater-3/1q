# check_doc_legacy_term_guard.cmake
#
# 守护规范性文档不残留已收口的旧入口术语(HARD 阻断)。
#
# 背景:公开 API 已从独立工厂类(SessionFactory)收口为静态成员
# (Session::Create / CreateWithValidation / CreateWithDecisionEngine),
# 决策 SPI 命名空间从 extension:: 收口为 session::。代码与安装/白名单
# 守护已很强,但规范性文档仍可能漂移回旧术语,误导后续维护者。
#
# 本守护扫描"规范性文档"(docs/ 顶层 + docs/sar/contracts + docs/migration),
# 禁止出现以下已收口入口:
#   - RadarSessionFactory::CreateWithDecisionEngine
#   - RadarSessionFactory::Create
#   - extension::ITacticalDecisionEngine  (现为 session::ITacticalDecisionEngine)
#   - EosSessionFactory / SarSessionFactory / EsrSessionFactory  (ghost 类,已删)
#
# 历史审查快照(docs/review/, docs/sar/audits/)已各自加 "historical snapshot"
# 标记,明确其内容为过程记录,故排除在扫描之外。

cmake_minimum_required(VERSION 3.16)

set(LEGACY_TERMS
    "RadarSessionFactory::CreateWithDecisionEngine"
    "RadarSessionFactory::Create"
    "extension::ITacticalDecisionEngine"
    "EosSessionFactory"
    "SarSessionFactory"
    "EsrSessionFactory"
)

# 规范性文档(显式列举,非递归扫 docs/,避免误入 review/audits 历史目录):
#   - docs/ 顶层 .md（合同、ADR、observability）
#   - docs/sar/contracts/、docs/migration/
set(NORMATIVE_DOC_FILES "")
file(GLOB _top_level_docs "${SOURCE_DIR}/docs/*.md")
list(APPEND NORMATIVE_DOC_FILES ${_top_level_docs})
file(GLOB_RECURSE _sar_contracts "${SOURCE_DIR}/docs/sar/contracts/*.md")
list(APPEND NORMATIVE_DOC_FILES ${_sar_contracts})
file(GLOB_RECURSE _migration_docs "${SOURCE_DIR}/docs/migration/*.md")
list(APPEND NORMATIVE_DOC_FILES ${_migration_docs})

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
        "若是历史审查记录,应放在 docs/review/ 或 docs/sar/audits/(已排除),\n"
        "并在文件顶部标注 'historical snapshot, not current API'。")
endif()

list(LENGTH VIOLATIONS _violation_count)
message(STATUS "[doc-legacy-term] 通过:规范性文档无旧入口术语残留 (${_violation_count} 处违规)。")
