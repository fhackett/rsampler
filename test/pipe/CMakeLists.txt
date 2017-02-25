
add_executable(pipe pipe.cpp)
target_link_libraries(pipe ${ARMADILLO_LIBRARIES} Threads::Threads ${LIBSAMPLERATE})

add_test(NAME pipe_test COMMAND pipe)
add_test(NAME pipe_valgrind_test COMMAND valgrind --error-exitcode=1 --leak-check=full $<TARGET_FILE:pipe>)
