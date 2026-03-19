function(goat_vesc_configure_docs)
  if(NOT GOAT_VESC_BUILD_DOCS)
    message(STATUS "API documentation target disabled (GOAT_VESC_BUILD_DOCS=OFF).")
    return()
  endif()

  find_package(Doxygen QUIET)
  if(NOT DOXYGEN_FOUND)
    message(STATUS "Doxygen not found; skipping API documentation target.")
    return()
  endif()

  set(GOAT_VESC_DOCS_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/docs")
  set(GOAT_VESC_DOXYFILE "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")

  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in"
    "${GOAT_VESC_DOXYFILE}"
    @ONLY
  )

  add_custom_target(docs
    COMMAND "${DOXYGEN_EXECUTABLE}" "${GOAT_VESC_DOXYFILE}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Generating goat_vesc API documentation"
    VERBATIM
  )

  message(STATUS "API documentation target enabled: cmake --build ${CMAKE_CURRENT_BINARY_DIR} --target docs")
endfunction()
