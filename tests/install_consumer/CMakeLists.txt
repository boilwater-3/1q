cmake_minimum_required(VERSION 3.16)

project(install_consumer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(1q CONFIG REQUIRED)

add_executable(default_session_consumer default_session_consumer.cpp)
target_link_libraries(default_session_consumer PRIVATE 1q::1q)

add_executable(advanced_extension_consumer advanced_extension_consumer.cpp)
target_link_libraries(advanced_extension_consumer PRIVATE 1q::1q)
