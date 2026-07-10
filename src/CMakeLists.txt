# 模块级清单：按子目录维护源文件与安装规则
if(ENABLE_INSTALL)
    include(GNUInstallDirs)
endif()
include(${CMAKE_CURRENT_SOURCE_DIR}/common/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/airborne_radar/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/electronic_surveillance_radar/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/electro_optical_sensor/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/sbirs_sensor/CMakeLists.txt)
# flight_dynamic (机动模块): 默认 OFF，仅在 ONEQ_ENABLE_FLIGHT_DYNAMIC 开启时参与编译
if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    include(${CMAKE_CURRENT_SOURCE_DIR}/flight_dynamic/CMakeLists.txt)
endif()
include(${CMAKE_CURRENT_SOURCE_DIR}/sar/CMakeLists.txt)

# --- OBJECT 库 ---
add_library(airborne_engine OBJECT ${AIRBORNE_ENGINE_SOURCES})
add_library(airborne_core OBJECT ${AIRBORNE_CORE_SOURCES})
add_library(esr_engine OBJECT ${ESR_ENGINE_SOURCES})
add_library(esr_core OBJECT ${ESR_CORE_SOURCES})
add_library(eos_engine OBJECT ${EOS_ENGINE_SOURCES})
add_library(eos_core OBJECT ${EOS_CORE_SOURCES})
add_library(sbirs_engine OBJECT ${SBIRS_ENGINE_SOURCES})
add_library(sbirs_core OBJECT ${SBIRS_CORE_SOURCES})
if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    add_library(fd_engine OBJECT ${FD_ENGINE_SOURCES})
endif()
add_library(sar_core OBJECT ${SAR_CORE_SOURCES})
if(SAR_ENGINE_SOURCES)
  add_library(sar_engine OBJECT ${SAR_ENGINE_SOURCES})
endif()
if(FD_CORE_SOURCES)
  add_library(fd_core OBJECT ${FD_CORE_SOURCES})
endif()

set(ONEQ_OBJECT_TARGETS
    airborne_engine
    airborne_core
    esr_engine
    esr_core
    eos_engine
    eos_core
    sbirs_engine
    sbirs_core
    sar_core
)

if(TARGET sar_engine)
  list(APPEND ONEQ_OBJECT_TARGETS sar_engine)
endif()

if(TARGET fd_engine)
  list(APPEND ONEQ_OBJECT_TARGETS fd_engine)
endif()

if(TARGET fd_core)
  list(APPEND ONEQ_OBJECT_TARGETS fd_core)
endif()

set(PROJECT_SOURCES
    ${ONEQ_COMMON_SOURCES}
    $<TARGET_OBJECTS:airborne_engine>
    $<TARGET_OBJECTS:airborne_core>
    $<TARGET_OBJECTS:esr_engine>
    $<TARGET_OBJECTS:esr_core>
    $<TARGET_OBJECTS:eos_engine>
    $<TARGET_OBJECTS:eos_core>
    $<TARGET_OBJECTS:sbirs_engine>
    $<TARGET_OBJECTS:sbirs_core>
    $<TARGET_OBJECTS:sar_core>
)

if(TARGET sar_engine)
  list(APPEND PROJECT_SOURCES $<TARGET_OBJECTS:sar_engine>)
endif()

if(TARGET fd_engine)
  list(APPEND PROJECT_SOURCES $<TARGET_OBJECTS:fd_engine>)
endif()

if(TARGET fd_core)
  list(APPEND PROJECT_SOURCES $<TARGET_OBJECTS:fd_core>)
endif()
# 追加 vendor OBJECT 目标（flatbuffers_objects, zlib_objects 等）
if(ONEQ_VENDOR_OBJECT_TARGETS)
    foreach(_vendor_obj IN LISTS ONEQ_VENDOR_OBJECT_TARGETS)
        list(APPEND PROJECT_SOURCES $<TARGET_OBJECTS:${_vendor_obj}>)
    endforeach()
    unset(_vendor_obj)
endif()

# --- 主库目标 ---
add_library(${PROJECT_CORE_TARGET} ${PROJECT_SOURCES})
add_library(${PROJECT_ALIAS} ALIAS ${PROJECT_CORE_TARGET})

include(GenerateExportHeader)
set(PROJECT_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/include")
set(PROJECT_GENERATED_EXPORT_HEADER
    "${PROJECT_GENERATED_INCLUDE_DIR}/1q/oneq_export.h")
file(MAKE_DIRECTORY "${PROJECT_GENERATED_INCLUDE_DIR}/1q")
generate_export_header(${PROJECT_CORE_TARGET}
    BASE_NAME ONEQ
    EXPORT_FILE_NAME "${PROJECT_GENERATED_EXPORT_HEADER}"
    STATIC_DEFINE ONEQ_STATIC_DEFINE
)

target_include_directories(${PROJECT_CORE_TARGET}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_GENERATED_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
foreach(ONEQ_OBJECT_TARGET IN LISTS ONEQ_OBJECT_TARGETS)
    target_include_directories(${ONEQ_OBJECT_TARGET}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${PROJECT_GENERATED_INCLUDE_DIR}
            ${CMAKE_SOURCE_DIR}/include
    )
endforeach()
unset(ONEQ_OBJECT_TARGET)

# vendor OBJECT 目标需要相同的 include 路径
if(ONEQ_VENDOR_OBJECT_TARGETS)
    foreach(ONEQ_OBJECT_TARGET IN LISTS ONEQ_VENDOR_OBJECT_TARGETS)
        target_include_directories(${ONEQ_OBJECT_TARGET}
            PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}
                ${PROJECT_GENERATED_INCLUDE_DIR}
                ${CMAKE_SOURCE_DIR}/include
        )
    endforeach()
    unset(ONEQ_OBJECT_TARGET)
endif()

# --- 第三方依赖 ---
include(${CMAKE_SOURCE_DIR}/cmake/project/ProjectDependencies.cmake)

# --- FlatBuffers 构建期代码生成 ---
# flatc 发现、schema 编译、生成头注入均由 FlatBuffersSetup 编排。
# 必须在 ProjectDependencies.cmake（find_package(flatbuffers)）之后、目标创建之后调用。
include(${CMAKE_SOURCE_DIR}/cmake/project/FlatBuffersSetup.cmake)

# --- 目标配置 ---
# 编译器选项、构建特性（Unity/Coverage/PCH）、目标属性（运行时库/输出名/可见性）
# 统一入口，对所有 OBJECT 目标与核心库目标一次性应用。
include(${CMAKE_SOURCE_DIR}/cmake/project/TargetConfiguration.cmake)

# --- 安装 ---
include(${CMAKE_SOURCE_DIR}/cmake/project/ProjectInstall.cmake)
