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

set(GOAT_VESC_SANITIZER_LIST "")
if(GOAT_VESC_ENABLE_ASAN OR GOAT_VESC_ENABLE_UBSAN)
  set(_goat_vesc_sanitizers "")
  if(GOAT_VESC_ENABLE_ASAN)
    list(APPEND _goat_vesc_sanitizers "address")
  endif()
  if(GOAT_VESC_ENABLE_UBSAN)
    list(APPEND _goat_vesc_sanitizers "undefined")
  endif()

  string(REPLACE ";" "," GOAT_VESC_SANITIZER_LIST "${_goat_vesc_sanitizers}")
endif()

function(goat_vesc_enable_sanitizers target_name)
  if(NOT GOAT_VESC_SANITIZER_LIST)
    return()
  endif()

  target_compile_options(${target_name} PRIVATE
    -fsanitize=${GOAT_VESC_SANITIZER_LIST}
    -fno-omit-frame-pointer
  )
  target_link_options(${target_name} PRIVATE
    -fsanitize=${GOAT_VESC_SANITIZER_LIST}
    -fno-omit-frame-pointer
  )
endfunction()

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
goat_vesc_enable_sanitizers(goat_vesc)

if(GOAT_VESC_BUILD_TOOLS)
  add_executable(vesc_probe tools/vesc_probe.cpp)
  target_link_libraries(vesc_probe PRIVATE goat_vesc)
  goat_vesc_enable_sanitizers(vesc_probe)
  add_executable(vesc_example tools/vesc_example.cpp)
  target_link_libraries(vesc_example PRIVATE goat_vesc)
  goat_vesc_enable_sanitizers(vesc_example)
  add_executable(vesc_duty_sweep tools/vesc_duty_sweep.cpp)
  target_link_libraries(vesc_duty_sweep PRIVATE goat_vesc)
  goat_vesc_enable_sanitizers(vesc_duty_sweep)
  add_executable(vesc_servo_sweep tools/vesc_servo_sweep.cpp)
  target_link_libraries(vesc_servo_sweep PRIVATE goat_vesc)
  goat_vesc_enable_sanitizers(vesc_servo_sweep)
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
