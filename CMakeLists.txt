cmake_minimum_required (VERSION 3.1)
project (astepcooler_test)

set (astepcooler_VERSION_MAJOR 1)
set (astepcooler_VERSION_MINOR 1)
set (astepcooler_VERSION_PATCH 1)

set (CMAKE_C_STANDARD 99)

configure_file(
  "${PROJECT_SOURCE_DIR}/src/astepcooler_test.h.in"
  "${PROJECT_SOURCE_DIR}/src/astepcooler_test.h"
  )

file(GLOB GLOB_SRC "src/*.h" "src/*.c" )

include_directories( "${PROJECT_SOURCE_DIR}/src" )

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable( astepcooler_test ${GLOB_SRC} ) 