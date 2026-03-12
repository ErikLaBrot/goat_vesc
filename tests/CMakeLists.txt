add_executable(test_interface test_interface.cpp)
target_link_libraries(test_interface PRIVATE goat_vesc)
add_test(NAME test_interface COMMAND test_interface) 
