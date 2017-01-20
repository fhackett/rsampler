
add_executable(simple simple.cpp)
target_link_libraries(simple Threads::Threads)

add_test(NAME simple_test COMMAND simple)

add_executable(double double.cpp)
target_link_libraries(double Threads::Threads)

add_test(NAME double_test COMMAND double)
