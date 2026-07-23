cmake_minimum_required(VERSION 3.21)
project(goat_vesc VERSION 0.1.0 LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(Threads REQUIRED)

option(GOAT_VESC_BUILD_EXAMPLES "Build examples" ON)
option(GOAT_VESC_ENABLE_CLANG_TIDY "Enable clang-tidy" OFF)
option(GOAT_VESC_ENABLE_CPPCHECK "Enable cppcheck" OFF)
option(GOAT_VESC_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(GOAT_VESC_ENABLE_TSAN "Enable ThreadSanitizer" OFF)
option(GOAT_VESC_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(GOAT_VESC_ENABLE_WERROR "Treat compiler warnings as errors" OFF)
option(GOAT_VESC_BUILD_DOCS "Enable API documentation target when Doxygen is available" ON)

include(cmake/StaticAnalysis.cmake)
include(cmake/Sanitizers.cmake)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(goat_vesc
  src/packet_parser.cpp
  src/protocol.cpp
  src/vesc_client.cpp
)
add_library(goat_vesc::goat_vesc ALIAS goat_vesc)

target_include_directories(goat_vesc
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(goat_vesc PUBLIC cxx_std_17)
target_link_libraries(goat_vesc PUBLIC Threads::Threads)

goat_vesc_enable_warnings(goat_vesc)
goat_vesc_enable_sanitizers(goat_vesc)

if(GOAT_VESC_BUILD_EXAMPLES)
  foreach(example IN ITEMS vesc_probe vesc_hardware_smoke)
    add_executable(${example} examples/${example}.cpp)
    target_link_libraries(${example} PRIVATE goat_vesc)
    goat_vesc_enable_warnings(${example})
    goat_vesc_enable_sanitizers(${example})
  endforeach()
endif()

if(BUILD_TESTING)
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
  COMPATIBILITY SameMinorVersion
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

if(GOAT_VESC_BUILD_DOCS)
  find_package(Doxygen QUIET)
  if(DOXYGEN_FOUND)
    set(GOAT_VESC_DOCS_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/docs")
    set(GOAT_VESC_DOXYFILE "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")
    configure_file(docs/Doxyfile.in "${GOAT_VESC_DOXYFILE}" @ONLY)
    add_custom_target(docs
      COMMAND "${DOXYGEN_EXECUTABLE}" "${GOAT_VESC_DOXYFILE}"
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      COMMENT "Generating goat_vesc API documentation"
      VERBATIM
    )
  else()
    message(STATUS "Doxygen not found; skipping API documentation target.")
  endif()
endif()
