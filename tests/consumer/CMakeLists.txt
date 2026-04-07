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

add_executable(eos_session_consumer eos_session_consumer.cpp)
target_link_libraries(eos_session_consumer PRIVATE 1q::1q)

add_executable(eos_extension_consumer eos_extension_consumer.cpp)
target_link_libraries(eos_extension_consumer PRIVATE 1q::1q)
