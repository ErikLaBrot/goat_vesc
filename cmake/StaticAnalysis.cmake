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
  endif()
endfunction()

if(GOAT_VESC_ENABLE_CLANG_TIDY)
  set(CMAKE_CXX_CLANG_TIDY clang-tidy --quiet)
  if(GOAT_VESC_ENABLE_WERROR)
    list(APPEND CMAKE_CXX_CLANG_TIDY --warnings-as-errors=*)
  endif()
endif()

if(GOAT_VESC_ENABLE_CPPCHECK)
  set(CMAKE_CXX_CPPCHECK
    cppcheck
    --enable=warning,style,performance,portability
    --inline-suppr
    --check-level=exhaustive
    --std=c++17
    --quiet
    --suppress=assertWithSideEffect
    --suppress=missingIncludeSystem
  )
  if(GOAT_VESC_ENABLE_WERROR)
    list(APPEND CMAKE_CXX_CPPCHECK --error-exitcode=2)
  endif()
endif()
