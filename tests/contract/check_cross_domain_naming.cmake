# 跨域命名规约守护：防止四域（airborne_radar / electro_optical_sensor /
# electronic_surveillance_radar / sar）在演进中重新引入"同概念异名"漂移。
#
# 本检查分两层：
#   - 阻断层（HARD）：已在本轮统一的项，一旦回退立即 FATAL_ERROR。
#       * 四域 Session/TraceSession 的 Step()/StepWithResult() 签名不得再出现
#         `session::` 相对限定或 `::域::session::` 全局根限定（类型定义在当前
#         session 命名空间内，裸名即可解析，多余限定是历史遗留，见
#         foundation/SensorContract.h 与本轮维度2 清理）。
#   - 报告层（SOFT）：尚未机械统一、需领域判断的项，仅打印告警供 review。
#       * 四域"最小 SNR 门限"字段名前缀目前有 minimum_/min_detect_/min_/min_valid_
#         四种；属步骤4 命名统一范围，此处只统计不阻断。
#
# 配套：跨域"形状契约"（Step/StepWithResult 返回类型）由编译期
# foundation/SensorContract.h 的 static_assert 守护，本脚本不重复检查类型形状。

if(NOT DEFINED SOURCE_DIR OR SOURCE_DIR STREQUAL "")
  message(FATAL_ERROR "SOURCE_DIR must be provided")
endif()

set(PUBLIC_INCLUDE_ROOT "${SOURCE_DIR}/include/1q")
set(SESSION_DIRS
    "${PUBLIC_INCLUDE_ROOT}/airborne_radar/session"
    "${PUBLIC_INCLUDE_ROOT}/electro_optical_sensor/session"
    "${PUBLIC_INCLUDE_ROOT}/electronic_surveillance_radar/session"
    "${PUBLIC_INCLUDE_ROOT}/sar/session")

set(VIOLATIONS)

# ---- 阻断层：Session 签名不得带多余命名空间限定 ----
foreach(SESSION_DIR IN LISTS SESSION_DIRS)
  file(GLOB SESSION_HEADERS "${SESSION_DIR}/*Session.h")
  foreach(HEADER IN LISTS SESSION_HEADERS)
    file(STRINGS "${HEADER}" ALL_LINES)
    set(_line_no 0)
    foreach(LINE IN LISTS ALL_LINES)
      math(EXPR _line_no "${_line_no} + 1")
      # 只看 Step()/StepWithResult() 声明行
      if(LINE MATCHES "(Step|StepWithResult)[ \t]*\\(")
        # 排除注释/doxygen 行
        string(STRIP "${LINE}" _stripped)
        if(_stripped MATCHES "^(//|/\\*|\\*)")
          continue()
        endif()
        # 返回类型若以 session:: 或 ::域::session:: 开头 → 违规
        if(LINE MATCHES "(session::|::[A-Za-z_]+::session::)[A-Za-z_]+[ \t]+(Step|StepWithResult)")
          list(APPEND VIOLATIONS
               "${HEADER}:${_line_no}: Session 签名不得带 session::/::域::session:: 限定（裸名即可解析）: ${LINE}")
        endif()
      endif()
    endforeach()
  endforeach()
endforeach()

if(VIOLATIONS)
  list(JOIN VIOLATIONS "\n" VIOLATION_TEXT)
  message(FATAL_ERROR
          "跨域命名规约守护失败（阻断层）。\n"
          "规则：四域 Session::Step()/StepWithResult() 签名必须使用裸类型名，\n"
          "      不得带 session:: 相对限定或 ::域::session:: 全局根限定。\n"
          "理由：相关类型定义在当前 session 命名空间内，裸名即可解析；多余限定是\n"
          "      历史遗留（曾因类型归位前的名字查找冲突而添加），现已无必要。\n"
          "见：foundation/SensorContract.h（形状契约）与本轮维度2 清理记录。\n"
          "违规：\n${VIOLATION_TEXT}")
endif()

# ---- 报告层：SNR 门限前缀异名统计（步骤4 范围，不阻断） ----
set(SNR_VARIANTS minimum_snr_db min_detect_snr_db min_snr_db min_valid_snr_db)
set(SNR_REPORT)
foreach(VARIANT IN LISTS SNR_VARIANTS)
  file(GLOB_RECURSE _matches
       LIST_DIRECTORIES FALSE
       "${PUBLIC_INCLUDE_ROOT}/*${VARIANT}*"
       "${SOURCE_DIR}/src/*${VARIANT}*")
  list(LENGTH _matches _count)
  if(_count GREATER 0)
    list(APPEND SNR_REPORT "  ${VARIANT}: ${_count} 处")
  endif()
endforeach()

if(SNR_REPORT)
  list(JOIN SNR_REPORT "\n" SNR_TEXT)
  message(STATUS
          "[跨域命名守护·报告层] 最小 SNR 门限字段存在多种前缀（步骤4 统一范围，本次不阻断）：\n"
          "${SNR_TEXT}\n"
          "建议统一为单一前缀（如 minimum_snr_db），减少跨域认知负担。")
endif()

message(STATUS "[跨域命名守护] 通过：四域 Session 签名裸名检查无违规。")
