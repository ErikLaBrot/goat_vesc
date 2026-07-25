if(GOAT_MOTOR_CONTROLLER_ENABLE_TSAN AND (GOAT_MOTOR_CONTROLLER_ENABLE_ASAN OR GOAT_MOTOR_CONTROLLER_ENABLE_UBSAN))
  message(FATAL_ERROR
    "ThreadSanitizer must run in a dedicated build. Disable ASAN and UBSAN "
    "when GOAT_MOTOR_CONTROLLER_ENABLE_TSAN=ON.")
endif()

set(GOAT_MOTOR_CONTROLLER_SANITIZER_LIST "")
set(GOAT_MOTOR_CONTROLLER_TEST_SANITIZER_ENVIRONMENT "")

if(GOAT_MOTOR_CONTROLLER_ENABLE_TSAN)
  set(GOAT_MOTOR_CONTROLLER_SANITIZER_LIST "thread")
elseif(GOAT_MOTOR_CONTROLLER_ENABLE_ASAN OR GOAT_MOTOR_CONTROLLER_ENABLE_UBSAN)
  set(_goat_motor_controller_sanitizers "")
  if(GOAT_MOTOR_CONTROLLER_ENABLE_ASAN)
    list(APPEND _goat_motor_controller_sanitizers "address")
  endif()
  if(GOAT_MOTOR_CONTROLLER_ENABLE_UBSAN)
    list(APPEND _goat_motor_controller_sanitizers "undefined")
  endif()

  string(REPLACE ";" "," GOAT_MOTOR_CONTROLLER_SANITIZER_LIST "${_goat_motor_controller_sanitizers}")
endif()

if(GOAT_MOTOR_CONTROLLER_ENABLE_UBSAN)
  list(APPEND GOAT_MOTOR_CONTROLLER_TEST_SANITIZER_ENVIRONMENT
    "UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1")
endif()

if(GOAT_MOTOR_CONTROLLER_ENABLE_TSAN)
  list(APPEND GOAT_MOTOR_CONTROLLER_TEST_SANITIZER_ENVIRONMENT
    "TSAN_OPTIONS=halt_on_error=1:abort_on_error=1")
endif()

function(goat_motor_controller_enable_sanitizers target_name)
  if(NOT GOAT_MOTOR_CONTROLLER_SANITIZER_LIST)
    return()
  endif()

  target_compile_options(${target_name} PRIVATE
    -fsanitize=${GOAT_MOTOR_CONTROLLER_SANITIZER_LIST}
    -fno-omit-frame-pointer
  )
  target_link_options(${target_name} PRIVATE
    -fsanitize=${GOAT_MOTOR_CONTROLLER_SANITIZER_LIST}
    -fno-omit-frame-pointer
  )
endfunction()

function(goat_motor_controller_configure_sanitizer_test test_name)
  if(NOT GOAT_MOTOR_CONTROLLER_TEST_SANITIZER_ENVIRONMENT)
    return()
  endif()

  set_tests_properties(${test_name} PROPERTIES
    ENVIRONMENT "${GOAT_MOTOR_CONTROLLER_TEST_SANITIZER_ENVIRONMENT}"
  )
endfunction()
