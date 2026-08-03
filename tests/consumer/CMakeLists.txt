cmake_minimum_required(VERSION 3.16)

project(consumer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(1q CONFIG REQUIRED)

add_executable(ar_session_consumer ar_session_consumer.cpp)
target_link_libraries(ar_session_consumer PRIVATE 1q::1q)

add_executable(ar_extension_consumer ar_extension_consumer.cpp)
target_link_libraries(ar_extension_consumer PRIVATE 1q::1q)

add_executable(esr_session_consumer esr_session_consumer.cpp)
target_link_libraries(esr_session_consumer PRIVATE 1q::1q)

add_executable(esr_extension_consumer esr_extension_consumer.cpp)
target_link_libraries(esr_extension_consumer PRIVATE 1q::1q)

add_executable(ecm_session_consumer ecm_session_consumer.cpp)
target_link_libraries(ecm_session_consumer PRIVATE 1q::1q)

add_executable(eos_session_consumer eos_session_consumer.cpp)
target_link_libraries(eos_session_consumer PRIVATE 1q::1q)

add_executable(eos_extension_consumer eos_extension_consumer.cpp)
target_link_libraries(eos_extension_consumer PRIVATE 1q::1q)

# 三层输出可观测性 consumer：验证 debug view / lifecycle recorder / attribution
# 在安装后可被外部工程编译链接（阶段 9 Batch F）。
add_executable(eos_output_observability_consumer eos_output_observability_consumer.cpp)
target_link_libraries(eos_output_observability_consumer PRIVATE 1q::1q)

# 注：esr_output_observability_consumer.cpp 在仓库中从未存在（悬空引用），
# 已从本工程移除；如需要 ESR 输出可观测性 consumer 请重新实现。

add_executable(sar_output_observability_consumer sar_output_observability_consumer.cpp)
target_link_libraries(sar_output_observability_consumer PRIVATE 1q::1q)
