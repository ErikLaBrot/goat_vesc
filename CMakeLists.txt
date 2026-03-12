cmake_minimum_required(VERSION 3.16)
project(goat_vesc VERSION 0.1.0 LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)

option(GOAT_VESC_BUILD_TESTS "Build tests" ON)
option(GOAT_VESC_BUILD_TOOLS "Build tools" ON)
option(GOAT_VESC_ENABLE_CLANG_TIDY "Enable clang-tidy" OFF)
option(GOAT_VESC_ENABLE_CPPCHECK "Enable cppcheck" OFF)
option(GOAT_VESC_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(GOAT_VESC_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(goat_vesc
  src/packet_builder.cpp
  src/packet_parser.cpp
  src/protocol.cpp
  src/types.cpp
  src/vesc_client.cpp
)

target_include_directories(goat_vesc
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_options(goat_vesc PRIVATE
  -Wall -Wextra -Wpedantic -Wconversion -Wshadow
)

if(GOAT_VESC_BUILD_TOOLS)
  add_executable(vesc_probe tools/vesc_probe.cpp)
  target_link_libraries(vesc_probe PRIVATE goat_vesc)
  add_executable(vesc_example tools/vesc_example.cpp)
  target_link_libraries(vesc_example PRIVATE goat_vesc)
endif()

if(GOAT_VESC_BUILD_TESTS)
  add_subdirectory(tests)
endif()

if(GOAT_VESC_ENABLE_CLANG_TIDY)
  set_target_properties(goat_vesc PROPERTIES
    CXX_CLANG_TIDY "clang-tidy"
  )
endif()

if(GOAT_VESC_ENABLE_CPPCHECK)
  set_target_properties(goat_vesc PROPERTIES
    CXX_CPPCHECK "cppcheck;--enable=warning,style,performance,portability;--std=c++17;--quiet"
  )
endif()

if(GOAT_VESC_ENABLE_ASAN OR GOAT_VESC_ENABLE_UBSAN)
  set(_san_flags "")
  if(GOAT_VESC_ENABLE_ASAN)
    list(APPEND _san_flags "address")
  endif()
  if(GOAT_VESC_ENABLE_UBSAN)
    list(APPEND _san_flags "undefined")
  endif()

  string(REPLACE ";" "," _san_list "${_san_flags}")
  target_compile_options(goat_vesc PRIVATE -fsanitize=${_san_list} -fno-omit-frame-pointer)
  target_link_options(goat_vesc PRIVATE -fsanitize=${_san_list} -fno-omit-frame-pointer)
endif()
