# 跨域命名规约守护：防止四域（airborne_radar / electro_optical_sensor /
# electronic_surveillance_radar / sar）在演进中重新引入"同概念异名"漂移。
#
# 本检查对"已统一"的项做 HARD 阻断（回退即 FATAL_ERROR）。这些项都经历过跨域
# 统一改造，再次出现旧名意味着回归：
#   1) 四域 Session/TraceSession 的 Step()/StepWithResult() 签名不得带 session::/
#      ::域::session:: 限定（维度2 统一为裸名）。
#   2) AR 工作模式不得再用 work_sub_mode 字段名或 RadarWorkSubMode enum 类型名
#      或 WithRadarWorkSubMode/WithRadarWork* 建造方法名（P2-b 统一为 work_mode /
#      RadarWorkMode / WithWorkMode，对齐 EOS/ESR）。
#   3) ESR/AR 运行期补丁的环境补丁槽不得再用 environment_runtime_config 字段名
#      或 has_environment_runtime_config / WithEnvironmentRuntimeConfig（P1-b 统一为
#      environment / has_environment / WithEnvironment，对齐 EOS）。
#   4) ESR/SAR 的最小 SNR 门限不得再用 min_detect_snr_db / min_valid_snr_db
#      前缀（P2-a 统一为 minimum_snr_db，对齐 EOS）。注意 AR 的 min_snr_db 因与
#      timing model 动态门限同名且语义不同，刻意不在本检查范围（见 P2-a 决策）。
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

# ---- 阻断 1：Session 签名不得带多余命名空间限定 ----
foreach(SESSION_DIR IN LISTS SESSION_DIRS)
  file(GLOB SESSION_HEADERS "${SESSION_DIR}/*Session.h")
  foreach(HEADER IN LISTS SESSION_HEADERS)
    file(STRINGS "${HEADER}" ALL_LINES)
    set(_line_no 0)
    foreach(LINE IN LISTS ALL_LINES)
      math(EXPR _line_no "${_line_no} + 1")
      if(LINE MATCHES "(Step|StepWithResult)[ \t]*\\(")
        string(STRIP "${LINE}" _stripped)
        if(_stripped MATCHES "^(//|/\\*|\\*)")
          continue()
        endif()
        if(LINE MATCHES "(session::|::[A-Za-z_]+::session::)[A-Za-z_]+[ \t]+(Step|StepWithResult)")
          list(APPEND VIOLATIONS
               "${HEADER}:${_line_no}: [签名限定] Session 签名不得带 session::/::域::session:: 限定: ${LINE}")
        endif()
      endif()
    endforeach()
  endforeach()
endforeach()

# ---- 阻断 2/3/4：跨域统一后的旧名不得回归 ----
# 扫描范围：公共头 + 域实现源码 + 测试。generated/ 与本脚本自身排除。
set(TOKEN_BANS
    "work_sub_mode|P2-b work_sub_mode 已统一为 work_mode"
    "RadarWorkSubMode|P2-b RadarWorkSubMode 已统一为 RadarWorkMode"
    "WithRadarWorkSubMode|P2-b WithRadarWorkSubMode 已统一为 WithWorkMode"
    "has_environment_runtime_config|P1-b has_environment_runtime_config 已统一为 has_environment"
    "environment_runtime_config|P1-b 补丁槽 environment_runtime_config 已统一为 environment（注意：类型名 EnvironmentRuntimeConfigPatch 保留，不在禁列）"
    "WithEnvironmentRuntimeConfig|P1-b WithEnvironmentRuntimeConfig 已统一为 WithEnvironment"
    "min_detect_snr_db|P2-a min_detect_snr_db 已统一为 minimum_snr_db"
    "min_valid_snr_db|P2-a min_valid_snr_db 已统一为 minimum_snr_db")

set(SCAN_DIRS
    "${PUBLIC_INCLUDE_ROOT}"
    "${SOURCE_DIR}/src/airborne_radar"
    "${SOURCE_DIR}/src/electronic_surveillance_radar"
    "${SOURCE_DIR}/src/sar"
    "${SOURCE_DIR}/src/electro_optical_sensor"
    "${SOURCE_DIR}/tests")

foreach(SCAN_DIR IN LISTS SCAN_DIRS)
  file(GLOB_RECURSE SCAN_FILES
       "${SCAN_DIR}/*.h" "${SCAN_DIR}/*.hpp" "${SCAN_DIR}/*.cpp" "${SCAN_DIR}/*.cmake")
  foreach(FILE_PATH IN LISTS SCAN_FILES)
    # 排除 generated 与本脚本
    if(FILE_PATH MATCHES "generated/" OR FILE_PATH MATCHES "check_cross_domain_naming")
      continue()
    endif()
    file(STRINGS "${FILE_PATH}" FILE_LINES)
    set(_line_no 0)
    foreach(LINE IN LISTS FILE_LINES)
      math(EXPR _line_no "${_line_no} + 1")
      foreach(BAN IN LISTS TOKEN_BANS)
        # 拆 pattern 与描述
        string(FIND "${BAN}" "|" _sep)
        math(EXPR _desc_start "${_sep} + 1")
        string(SUBSTRING "${BAN}" 0 ${_sep} _pattern)
        string(SUBSTRING "${BAN}" ${_desc_start} -1 _desc)
        # 跳过含旧名的说明性注释行（如 "已统一为 work_mode"），避免误伤迁移说明
        string(STRIP "${LINE}" _stripped)
        if(_stripped MATCHES "^(//|\\*)")
          continue()
        endif()
        # 注意：environment_runtime_config 会匹配 EsrEnvironmentRuntimeConfigPatch
        # 这类类型名 —— 刻意保留这类类型名，用大小写区分（小写才禁）。
        string(FIND "${LINE}" "${_pattern}" _pos)
        if(NOT _pos EQUAL -1)
          list(APPEND VIOLATIONS "${FILE_PATH}:${_line_no}: [${_desc}]: ${LINE}")
        endif()
      endforeach()
    endforeach()
  endforeach()
endforeach()

if(VIOLATIONS)
  list(JOIN VIOLATIONS "\n" VIOLATION_TEXT)
  message(FATAL_ERROR
          "跨域命名规约守护失败。\n"
          "规则：下列项已跨域统一，旧名不得回归（详见各项说明）。\n"
          "见：foundation/SensorContract.h（形状契约）、docs 中的统一记录。\n"
          "违规：\n${VIOLATION_TEXT}")
endif()

message(STATUS "[跨域命名守护] 通过：Session 签名裸名 + work_mode + 补丁槽 + SNR 前缀均无回退。")
