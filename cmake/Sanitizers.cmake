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
