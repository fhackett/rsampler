cmake_minimum_required(VERSION 3.4)

project(rsampler)

enable_testing()

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)
find_package(Armadillo REQUIRED)

find_library(LIBJACK jack)
find_library(LIBSAMPLERATE samplerate)
find_library(LIBSNDFILE sndfile)

include_directories(
  include
  deps/range-v3/include
  ${ARMADILLO_INCLUDE_DIRS}
)

set(LIBS ${LIBJACK} ${LIBSAMPLERATE} ${LIBSNDFILE})

set(CMAKE_CXX_STANDARD "14")
set(CMAKE_CXX_STANDARD_REQUIRED YES)
#add_compile_options(--std=c++1z)

add_subdirectory(test)
