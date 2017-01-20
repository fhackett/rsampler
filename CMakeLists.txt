cmake_minimum_required(VERSION 3.4)

enable_testing()

include_directories(include)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

find_library(LIBJACK jack)
find_library(LIBSAMPLERATE samplerate)
find_library(LIBSNDFILE sndfile)

set(LIBS ${LIBJACK} ${LIBSAMPLERATE} ${LIBSNDFILE})

#set(CMAKE_CXX_STANDARD "14" CACHE INTERNAL "Enable C++14")
add_compile_options(--std=c++1z)

include(CMakeToolsHelpers OPTIONAL)

add_subdirectory(test)
