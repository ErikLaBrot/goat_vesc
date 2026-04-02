cmake_minimum_required(VERSION 3.16)
project(goat_vesc VERSION 0.1.0 LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(GOAT_VESC_BUILD_TESTS "Build tests" ON)
option(GOAT_VESC_BUILD_EXAMPLES "Build examples" ON)
option(GOAT_VESC_ENABLE_CLANG_TIDY "Enable clang-tidy" OFF)
option(GOAT_VESC_ENABLE_CPPCHECK "Enable cppcheck" OFF)
option(GOAT_VESC_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(GOAT_VESC_ENABLE_LSAN "Enable LeakSanitizer" OFF)
option(GOAT_VESC_ENABLE_TSAN "Enable ThreadSanitizer" OFF)
option(GOAT_VESC_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(GOAT_VESC_ENABLE_WERROR "Treat compiler warnings as errors" OFF)
option(GOAT_VESC_ENABLE_STRICT_MODE "Enable the repository quality gate defaults" OFF)
option(GOAT_VESC_CLANG_TIDY_WARNINGS_AS_ERRORS "Treat clang-tidy findings as errors" OFF)
option(GOAT_VESC_CPPCHECK_EXHAUSTIVE "Use cppcheck exhaustive mode" OFF)
option(GOAT_VESC_BUILD_DOCS "Enable API documentation target when Doxygen is available" ON)

if(GOAT_VESC_ENABLE_STRICT_MODE)
  set(GOAT_VESC_ENABLE_CLANG_TIDY ON CACHE BOOL "Enable clang-tidy" FORCE)
  set(GOAT_VESC_ENABLE_CPPCHECK ON CACHE BOOL "Enable cppcheck" FORCE)
  set(GOAT_VESC_ENABLE_WERROR ON CACHE BOOL "Treat compiler warnings as errors" FORCE)
  set(GOAT_VESC_CLANG_TIDY_WARNINGS_AS_ERRORS ON CACHE BOOL
      "Treat clang-tidy findings as errors" FORCE)
  set(GOAT_VESC_CPPCHECK_EXHAUSTIVE ON CACHE BOOL "Use cppcheck exhaustive mode" FORCE)
endif()

include(cmake/StaticAnalysis.cmake)
include(cmake/Sanitizers.cmake)
include(cmake/Docs.cmake)

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
add_library(goat_vesc::goat_vesc ALIAS goat_vesc)

target_include_directories(goat_vesc
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

goat_vesc_enable_warnings(goat_vesc)
goat_vesc_enable_sanitizers(goat_vesc)
goat_vesc_enable_static_analysis(goat_vesc)

if(GOAT_VESC_BUILD_EXAMPLES)
  add_executable(vesc_example examples/vesc_example.cpp)
  target_link_libraries(vesc_example PRIVATE goat_vesc)
  goat_vesc_enable_sanitizers(vesc_example)
  goat_vesc_enable_static_analysis(vesc_example)
  add_executable(vesc_probe examples/vesc_probe.cpp)
  target_link_libraries(vesc_probe PRIVATE goat_vesc)
  goat_vesc_enable_sanitizers(vesc_probe)
  goat_vesc_enable_static_analysis(vesc_probe)
  add_executable(vesc_duty_sweep examples/vesc_duty_sweep.cpp)
  target_link_libraries(vesc_duty_sweep PRIVATE goat_vesc)
  goat_vesc_enable_sanitizers(vesc_duty_sweep)
  goat_vesc_enable_static_analysis(vesc_duty_sweep)
  add_executable(vesc_servo_sweep examples/vesc_servo_sweep.cpp)
  target_link_libraries(vesc_servo_sweep PRIVATE goat_vesc)
  goat_vesc_enable_sanitizers(vesc_servo_sweep)
  goat_vesc_enable_static_analysis(vesc_servo_sweep)
  add_executable(vesc_hardware_smoke examples/vesc_hardware_smoke.cpp)
  target_link_libraries(vesc_hardware_smoke PRIVATE goat_vesc)
  goat_vesc_enable_sanitizers(vesc_hardware_smoke)
  goat_vesc_enable_static_analysis(vesc_hardware_smoke)
endif()

if(GOAT_VESC_BUILD_TESTS)
  add_subdirectory(tests)
endif()

set(GOAT_VESC_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")
set(GOAT_VESC_INSTALL_DATADIR "${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}")

install(TARGETS goat_vesc
  EXPORT goat_vescTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

configure_package_config_file(
  cmake/goat_vescConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/goat_vescConfig.cmake
  INSTALL_DESTINATION ${GOAT_VESC_INSTALL_CMAKEDIR}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/goat_vescConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(EXPORT goat_vescTargets
  FILE goat_vescTargets.cmake
  NAMESPACE goat_vesc::
  DESTINATION ${GOAT_VESC_INSTALL_CMAKEDIR}
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/goat_vescConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/goat_vescConfigVersion.cmake
  DESTINATION ${GOAT_VESC_INSTALL_CMAKEDIR}
)

install(FILES package.xml
  DESTINATION ${GOAT_VESC_INSTALL_DATADIR}
)

goat_vesc_configure_docs()
