cmake_minimum_required(VERSION 3.4)

include_directories(include)

find_library(LIBJACK jack)
find_library(LIBSAMPLERATE samplerate)
find_library(LIBSNDFILE sndfile)

#set(CMAKE_CXX_STANDARD "14" CACHE INTERNAL "Enable C++14")
add_compile_options(--std=c++1z)

add_executable(tmp test/tmp.cpp)
add_executable(basic test/basic.cpp)
target_link_libraries(basic ${LIBJACK} ${LIBSAMPLERATE} ${LIBSNDFILE})

add_executable(offline_test test/offline_test.cpp)
target_link_libraries(offline_test ${LIBJACK} ${LIBSAMPLERATE} ${LIBSNDFILE})
