if(GOAT_VESC_ENABLE_TSAN AND
   (GOAT_VESC_ENABLE_ASAN OR GOAT_VESC_ENABLE_LSAN OR GOAT_VESC_ENABLE_UBSAN))
  message(FATAL_ERROR
    "ThreadSanitizer must run in a dedicated build. Disable ASAN, LSAN, and UBSAN "
    "when GOAT_VESC_ENABLE_TSAN=ON.")
endif()

set(GOAT_VESC_SANITIZER_LIST "")
set(GOAT_VESC_TEST_SANITIZER_ENVIRONMENT "")

if(GOAT_VESC_ENABLE_TSAN)
  set(GOAT_VESC_SANITIZER_LIST "thread")
elseif(GOAT_VESC_ENABLE_ASAN OR GOAT_VESC_ENABLE_LSAN OR GOAT_VESC_ENABLE_UBSAN)
  set(_goat_vesc_sanitizers "")
  if(GOAT_VESC_ENABLE_ASAN)
    list(APPEND _goat_vesc_sanitizers "address")
  endif()
  if(GOAT_VESC_ENABLE_LSAN AND NOT GOAT_VESC_ENABLE_ASAN)
    list(APPEND _goat_vesc_sanitizers "leak")
  endif()
  if(GOAT_VESC_ENABLE_UBSAN)
    list(APPEND _goat_vesc_sanitizers "undefined")
  endif()

  string(REPLACE ";" "," GOAT_VESC_SANITIZER_LIST "${_goat_vesc_sanitizers}")
endif()

if(GOAT_VESC_ENABLE_ASAN)
  if(GOAT_VESC_ENABLE_LSAN)
    list(APPEND GOAT_VESC_TEST_SANITIZER_ENVIRONMENT
      "ASAN_OPTIONS=detect_leaks=1:halt_on_error=1")
  else()
    list(APPEND GOAT_VESC_TEST_SANITIZER_ENVIRONMENT
      "ASAN_OPTIONS=detect_leaks=0:halt_on_error=1")
  endif()
endif()

if(GOAT_VESC_ENABLE_LSAN AND NOT GOAT_VESC_ENABLE_ASAN)
  list(APPEND GOAT_VESC_TEST_SANITIZER_ENVIRONMENT
    "LSAN_OPTIONS=exitcode=23:print_suppressions=0")
endif()

if(GOAT_VESC_ENABLE_UBSAN)
  list(APPEND GOAT_VESC_TEST_SANITIZER_ENVIRONMENT
    "UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1")
endif()

if(GOAT_VESC_ENABLE_TSAN)
  list(APPEND GOAT_VESC_TEST_SANITIZER_ENVIRONMENT
    "TSAN_OPTIONS=halt_on_error=1:abort_on_error=1")
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

function(goat_vesc_configure_sanitizer_test test_name)
  if(NOT GOAT_VESC_TEST_SANITIZER_ENVIRONMENT)
    return()
  endif()

  set_tests_properties(${test_name} PROPERTIES
    ENVIRONMENT "${GOAT_VESC_TEST_SANITIZER_ENVIRONMENT}"
  )
endfunction()
