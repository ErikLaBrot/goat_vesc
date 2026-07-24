cmake_minimum_required(VERSION 3.21)
project(goat_motor_controller VERSION 0.1.0 LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(Threads REQUIRED)

option(GOAT_MOTOR_CONTROLLER_BUILD_EXAMPLES "Build examples" ON)
option(GOAT_MOTOR_CONTROLLER_ENABLE_CLANG_TIDY "Enable clang-tidy" OFF)
option(GOAT_MOTOR_CONTROLLER_ENABLE_CPPCHECK "Enable cppcheck" OFF)
option(GOAT_MOTOR_CONTROLLER_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(GOAT_MOTOR_CONTROLLER_ENABLE_TSAN "Enable ThreadSanitizer" OFF)
option(GOAT_MOTOR_CONTROLLER_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(GOAT_MOTOR_CONTROLLER_ENABLE_WERROR "Treat compiler warnings as errors" OFF)
option(GOAT_MOTOR_CONTROLLER_BUILD_DOCS "Enable API documentation target when Doxygen is available" ON)

include(cmake/StaticAnalysis.cmake)
include(cmake/Sanitizers.cmake)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(goat_motor_controller
  src/packet_parser.cpp
  src/protocol.cpp
  src/controller_client.cpp
)
add_library(goat_motor_controller::goat_motor_controller ALIAS goat_motor_controller)

target_include_directories(goat_motor_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(goat_motor_controller PUBLIC cxx_std_17)
target_link_libraries(goat_motor_controller PUBLIC Threads::Threads)

goat_motor_controller_enable_warnings(goat_motor_controller)
goat_motor_controller_enable_sanitizers(goat_motor_controller)

if(GOAT_MOTOR_CONTROLLER_BUILD_EXAMPLES)
  foreach(example IN ITEMS controller_probe controller_hardware_smoke)
    add_executable(${example} examples/${example}.cpp)
    target_link_libraries(${example} PRIVATE goat_motor_controller)
    goat_motor_controller_enable_warnings(${example})
    goat_motor_controller_enable_sanitizers(${example})
  endforeach()
endif()

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

set(GOAT_MOTOR_CONTROLLER_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")
set(GOAT_MOTOR_CONTROLLER_INSTALL_DATADIR "${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}")

install(TARGETS goat_motor_controller
  EXPORT goat_motor_controllerTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

configure_package_config_file(
  cmake/goat_motor_controllerConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/goat_motor_controllerConfig.cmake
  INSTALL_DESTINATION ${GOAT_MOTOR_CONTROLLER_INSTALL_CMAKEDIR}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/goat_motor_controllerConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMinorVersion
)

install(EXPORT goat_motor_controllerTargets
  FILE goat_motor_controllerTargets.cmake
  NAMESPACE goat_motor_controller::
  DESTINATION ${GOAT_MOTOR_CONTROLLER_INSTALL_CMAKEDIR}
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/goat_motor_controllerConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/goat_motor_controllerConfigVersion.cmake
  DESTINATION ${GOAT_MOTOR_CONTROLLER_INSTALL_CMAKEDIR}
)

install(FILES package.xml LICENSE
  DESTINATION ${GOAT_MOTOR_CONTROLLER_INSTALL_DATADIR}
)

if(GOAT_MOTOR_CONTROLLER_BUILD_DOCS)
  find_package(Doxygen QUIET)
  if(DOXYGEN_FOUND)
    set(GOAT_MOTOR_CONTROLLER_DOCS_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/docs")
    set(GOAT_MOTOR_CONTROLLER_DOXYFILE "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")
    configure_file(docs/Doxyfile.in "${GOAT_MOTOR_CONTROLLER_DOXYFILE}" @ONLY)
    add_custom_target(docs
      COMMAND "${DOXYGEN_EXECUTABLE}" "${GOAT_MOTOR_CONTROLLER_DOXYFILE}"
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      COMMENT "Generating goat_motor_controller API documentation"
      VERBATIM
    )
  else()
    message(STATUS "Doxygen not found; skipping API documentation target.")
  endif()
endif()
