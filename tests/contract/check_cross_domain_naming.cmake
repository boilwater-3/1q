# 跨域命名规约守护：防止四域（airborne_radar / electro_optical_sensor /
# electronic_surveillance_radar / sar）在演进中重新引入"同概念异名"漂移。
#
# 本检查对"已统一"的项做 HARD 阻断（回退即 FATAL_ERROR）。这些项都经历过跨域
# 统一改造，再次出现旧名意味着回归：
#   1) 四域 Session/TraceSession 的 Step()/StepWithResult() 签名不得带 session::/
#      ::域::session:: 限定（维度2 统一为裸名）。
#   2) AR 工作模式不得再用 work_sub_mode 字段名或 RadarWorkSubMode enum 类型名
#      或 WithRadarWorkSubMode/WithRadarWork* 建造方法名（P2-b 统一为 work_mode /
#      ArWorkMode / WithWorkMode，对齐 EOS/ESR）。
#   3) ESR/AR 运行期补丁的环境补丁槽不得再用 environment_runtime_config 字段名
#      或 has_environment_runtime_config / WithEnvironmentRuntimeConfig（P1-b 统一为
#      environment / has_environment / WithEnvironment，对齐 EOS）。
#   4) 四域 public 配置的最小 SNR 门限统一为 minimum_snr_db；旧的
#      min_detect_snr_db / min_valid_snr_db / min_snr_db 均不得回流。内部 timing
#      model 的动态门限不属于 public 配置命名契约。
#   5) 四域 RuntimeConfigBuilder 的链式方法统一以 With* 动词开头，不得回归
#      Set*/Enable* 旧动词（P3-b 统一）。仅约束 *RuntimeConfigBuilder.h 公共头，
#      不影响 ArSessionConfigBuilder::MissionEditor 等其它建造者类。
#   6) 四域 RuntimeConfigBuilder 必须提供 WithRuntimeConfigPatch 整块覆盖入口
#      （P3-b 对齐，四域形状一致）。
#   7) SBIRS mission、runtime patch 与 builder 的电源名称必须统一为 power_on。
#   8) EOS/SBIRS 探测器面积必须保留显式单位后缀；禁止退化为无单位字段名。
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
    "RadarWorkSubMode|P2-b RadarWorkSubMode 已统一为 ArWorkMode"
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

# public 配置统一使用 minimum_snr_db。此规则只扫描 include/1q 下的配置头，
# 避免误伤内部 timing model 等具有不同责任的局部参数。
file(GLOB_RECURSE PUBLIC_CONFIG_HEADERS
     "${PUBLIC_INCLUDE_ROOT}/airborne_radar/config/*.h"
     "${PUBLIC_INCLUDE_ROOT}/electro_optical_sensor/config/*.h"
     "${PUBLIC_INCLUDE_ROOT}/electronic_surveillance_radar/config/*.h"
     "${PUBLIC_INCLUDE_ROOT}/sar/config/*.h")
foreach(HEADER IN LISTS PUBLIC_CONFIG_HEADERS)
  file(STRINGS "${HEADER}" HEADER_LINES)
  set(_line_no 0)
  foreach(LINE IN LISTS HEADER_LINES)
    math(EXPR _line_no "${_line_no} + 1")
    string(STRIP "${LINE}" _stripped)
    if(_stripped MATCHES "^(//|/\\*|\\*)")
      continue()
    endif()
    if(LINE MATCHES "(^|[^A-Za-z0-9_])min_snr_db([^A-Za-z0-9_]|$)")
      list(APPEND VIOLATIONS
           "${HEADER}:${_line_no}: [public 最低 SNR 字段必须命名为 minimum_snr_db]: ${LINE}")
    endif()
  endforeach()
endforeach()

# ---- 阻断 5：RuntimeConfigBuilder 链式方法统一 With* 动词 ----
# 仅扫描四域 *RuntimeConfigBuilder.h 公共头，避免误伤 SessionConfigBuilder 等
# 其它合法持有 Set*/Enable* 动词的建造者类（如 ArSessionConfigBuilder::MissionEditor）。
set(RUNTIME_BUILDER_HEADERS
    "${PUBLIC_INCLUDE_ROOT}/airborne_radar/config/ArRuntimeConfigBuilder.h"
    "${PUBLIC_INCLUDE_ROOT}/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
    "${PUBLIC_INCLUDE_ROOT}/electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h"
    "${PUBLIC_INCLUDE_ROOT}/sar/config/SarRuntimeConfigBuilder.h")
# 禁止的链式方法动词前缀：Set*/Enable*（已统一为 With*）。
# 显式排除注释行与 Build()。Bare token 用正则词边界匹配声明形式
# "& Set...(" / "& Enable...("。
set(BANNED_VERBS_REGEX "(Builder&[ \t]+(Set|Enable)[A-Za-z0-9_]+[ \t]*\\()")

# ---- 阻断 6：四域 RuntimeConfigBuilder 必须有 WithRuntimeConfigPatch ----
foreach(HEADER IN LISTS RUNTIME_BUILDER_HEADERS)
  file(STRINGS "${HEADER}" HEADER_LINES)
  set(_line_no 0)
  set(_has_patch_entry FALSE)
  foreach(LINE IN LISTS HEADER_LINES)
    math(EXPR _line_no "${_line_no} + 1")
    string(STRIP "${LINE}" _stripped)
    # 跳过注释行
    if(_stripped MATCHES "^(//|/\\*|\\*)")
      continue()
    endif()
    # 阻断 5：禁止 Set*/Enable* 链式动词
    if(LINE MATCHES "${BANNED_VERBS_REGEX}")
      list(APPEND VIOLATIONS
           "${HEADER}:${_line_no}: [P3-b 动词统一] RuntimeConfigBuilder 链式方法须用 With*，禁止 Set*/Enable*: ${LINE}")
    endif()
    # 阻断 6：必须有 WithRuntimeConfigPatch 声明
    if(LINE MATCHES "WithRuntimeConfigPatch[ \t]*\\(")
      set(_has_patch_entry TRUE)
    endif()
  endforeach()
  if(NOT _has_patch_entry)
    list(APPEND VIOLATIONS
         "${HEADER}: [P3-b 整块入口] 缺少 WithRuntimeConfigPatch 声明，四域 RuntimeConfigBuilder 须提供整块覆盖入口")
  endif()
endforeach()

# ---- 阻断 7：SBIRS 电源命名唯一权威 ----
set(SBIRS_MISSION_HEADER
    "${PUBLIC_INCLUDE_ROOT}/sbirs_sensor/config/SbirsMissionConfig.h")
file(READ "${SBIRS_MISSION_HEADER}" SBIRS_MISSION_TEXT)
set(SBIRS_RUNTIME_PATCH_HEADER
    "${PUBLIC_INCLUDE_ROOT}/sbirs_sensor/config/SbirsRuntimeConfigPatch.h")
set(SBIRS_RUNTIME_BUILDER_HEADER
    "${PUBLIC_INCLUDE_ROOT}/sbirs_sensor/config/SbirsRuntimeConfigBuilder.h")
file(READ "${SBIRS_RUNTIME_PATCH_HEADER}" SBIRS_RUNTIME_PATCH_TEXT)
file(READ "${SBIRS_RUNTIME_BUILDER_HEADER}" SBIRS_RUNTIME_BUILDER_TEXT)
string(FIND "${SBIRS_MISSION_TEXT}" "bool power_on" _mission_power_pos)
string(FIND "${SBIRS_RUNTIME_PATCH_TEXT}" "bool has_power_on" _patch_power_pos)
string(FIND "${SBIRS_RUNTIME_BUILDER_TEXT}" "WithPowerOn" _builder_power_pos)
if(_mission_power_pos EQUAL -1 OR _patch_power_pos EQUAL -1 OR _builder_power_pos EQUAL -1)
  list(APPEND VIOLATIONS
       "[SBIRS 电源命名] mission、runtime patch 与 builder 必须统一提供 power_on")
endif()
string(FIND "${SBIRS_MISSION_TEXT}" "sensor_enabled" _mission_legacy_pos)
string(FIND "${SBIRS_RUNTIME_PATCH_TEXT}" "sensor_enabled" _patch_legacy_pos)
string(FIND "${SBIRS_RUNTIME_BUILDER_TEXT}" "sensor_enabled" _builder_legacy_pos)
if(NOT _mission_legacy_pos EQUAL -1 OR NOT _patch_legacy_pos EQUAL -1 OR
   NOT _builder_legacy_pos EQUAL -1)
  list(APPEND VIOLATIONS "[SBIRS 电源命名] 禁止回流旧名 sensor_enabled")
endif()

# ---- 阻断 8：跨域探测器面积单位后缀 ----
set(DETECTOR_AREA_HEADERS
    "${PUBLIC_INCLUDE_ROOT}/electro_optical_sensor/config/EosHardwareConfig.h|detector_area_cm2"
    "${PUBLIC_INCLUDE_ROOT}/sbirs_sensor/config/SbirsHardwareConfig.h|detector_area_m2")
foreach(HEADER_AND_TOKEN IN LISTS DETECTOR_AREA_HEADERS)
  string(FIND "${HEADER_AND_TOKEN}" "|" _separator)
  math(EXPR _token_start "${_separator} + 1")
  string(SUBSTRING "${HEADER_AND_TOKEN}" 0 ${_separator} _header)
  string(SUBSTRING "${HEADER_AND_TOKEN}" ${_token_start} -1 _required_token)
  file(READ "${_header}" _header_text)
  string(FIND "${_header_text}" "${_required_token}" _required_pos)
  if(_required_pos EQUAL -1)
    list(APPEND VIOLATIONS "${_header}: [探测器面积单位] 缺少 '${_required_token}'；"
                           "跨域同物理量必须在字段名中显式标明单位")
  endif()
  if(_header_text MATCHES "float[ \t]+detector_area[ \t]*[{;]")
    list(APPEND VIOLATIONS "${_header}: [探测器面积单位] 禁止无单位字段 detector_area；"
                           "使用模块冻结的 _cm2 或 _m2 后缀")
  endif()
endforeach()

if(VIOLATIONS)
  list(JOIN VIOLATIONS "\n" VIOLATION_TEXT)
  message(FATAL_ERROR
          "跨域命名规约守护失败。\n"
          "规则：下列项已跨域统一，旧名不得回归（详见各项说明）。\n"
          "见：foundation/SensorContract.h（形状契约）、docs 中的统一记录。\n"
          "违规：\n${VIOLATION_TEXT}")
endif()

message(STATUS
        "[跨域命名守护] 通过：Session 签名裸名 + work_mode + power_on + 补丁槽 + "
        "SNR 前缀 + Builder 动词/整块入口 + 探测器面积单位后缀均无回退。")
