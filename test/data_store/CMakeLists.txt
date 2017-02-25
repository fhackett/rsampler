
add_executable(basic basic.cpp)
target_link_libraries(basic Threads::Threads)

add_test(NAME basic_test COMMAND basic)
add_test(NAME basic_valgrind_test COMMAND valgrind --error-exitcode=1 --leak-check=full $<TARGET_FILE:basic>)
