cmake_minimum_required(VERSION 3.0.0)
project(dijkstra VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
include_directories("include/")

enable_testing()

add_executable(dijkstra main.cpp
src/Dijkstra.cpp
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
