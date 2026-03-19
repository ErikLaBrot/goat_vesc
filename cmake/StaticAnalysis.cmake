function(goat_vesc_enable_warnings target_name)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target_name} PRIVATE
      -Wall
      -Wextra
      -Wpedantic
      -Wconversion
      -Wshadow
      -Wdouble-promotion
      -Wformat=2
      -Wnull-dereference
      -Wold-style-cast
      -Woverloaded-virtual
      -Wundef
    )

    if(GOAT_VESC_ENABLE_WERROR)
      target_compile_options(${target_name} PRIVATE -Werror)
    endif()
  elseif(MSVC)
    target_compile_options(${target_name} PRIVATE /W4)

    if(GOAT_VESC_ENABLE_WERROR)
      target_compile_options(${target_name} PRIVATE /WX)
    endif()
  endif()
endfunction()

function(goat_vesc_enable_static_analysis target_name)
  if(GOAT_VESC_ENABLE_CLANG_TIDY)
    set(_goat_vesc_clang_tidy
      clang-tidy
      --quiet
    )

    if(GOAT_VESC_CLANG_TIDY_WARNINGS_AS_ERRORS)
      list(APPEND _goat_vesc_clang_tidy --warnings-as-errors=*)
    endif()

    set_target_properties(${target_name} PROPERTIES
      CXX_CLANG_TIDY "${_goat_vesc_clang_tidy}"
    )
  endif()

  if(GOAT_VESC_ENABLE_CPPCHECK)
    set(_goat_vesc_cppcheck
      cppcheck
      --enable=warning,style,performance,portability
      --inline-suppr
      --std=c++17
      --quiet
      --suppress=assertWithSideEffect
      --suppress=missingIncludeSystem
    )

    if(GOAT_VESC_CPPCHECK_EXHAUSTIVE)
      list(APPEND _goat_vesc_cppcheck --check-level=exhaustive)
    endif()

    if(GOAT_VESC_ENABLE_WERROR)
      list(APPEND _goat_vesc_cppcheck --error-exitcode=2)
    endif()

    set_target_properties(${target_name} PROPERTIES
      CXX_CPPCHECK "${_goat_vesc_cppcheck}"
    )
  endif()
endfunction()
