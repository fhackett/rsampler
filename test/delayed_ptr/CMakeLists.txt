
add_executable(simple simple.cpp)
target_link_libraries(simple Threads::Threads)

add_test(NAME simple_test COMMAND simple)
add_test(NAME simple_valgrind_test COMMAND valgrind --error-exitcode=1 --leak-check=full $<TARGET_FILE:simple>)

add_executable(double double.cpp)
target_link_libraries(double Threads::Threads)

add_test(NAME double_test COMMAND double)
add_test(NAME double_valgrind_test COMMAND valgrind --error-exitcode=1 --leak-check=full $<TARGET_FILE:double>)

add_executable(atomic atomic.cpp)
target_link_libraries(atomic Threads::Threads)

add_test(NAME atomic_test COMMAND atomic)
add_test(NAME atomic_valgrind_test COMMAND valgrind --error-exitcode=1 --leak-check=full $<TARGET_FILE:atomic>)
