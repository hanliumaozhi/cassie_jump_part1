cmake_minimum_required(VERSION 3.17)
project(cassie_jump_part1)

option(RUN_X11_TESTS "Run tests that require X11" OFF)

include(CTest)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_C_COMPILER /usr/bin/gcc-10)
set(CMAKE_CXX_COMPILER /usr/bin/g++-10)


if(APPLE)
    set(FIND_PYTHON_EXECUTABLE_PATHS /usr/local/opt/python@3.8/bin)
    set(FIND_PYTHON_INTERP_VERSION 3.8)
else()
    set(FIND_PYTHON_EXECUTABLE_PATHS /usr/bin)
    set(FIND_PYTHON_INTERP_VERSION 3.6)
endif()
find_program(PYTHON_EXECUTABLE NAMES python3
        PATHS "${FIND_PYTHON_EXECUTABLE_PATHS}"
        NO_DEFAULT_PATH
        )
find_package(PythonInterp ${FIND_PYTHON_INTERP_VERSION} MODULE REQUIRED)

execute_process(COMMAND ${PYTHON_EXECUTABLE}-config --exec-prefix
        OUTPUT_VARIABLE PYTHON_EXEC_PREFIX
        OUTPUT_STRIP_TRAILING_WHITESPACE
        )

set(CMAKE_PREFIX_PATH "/opt/drake")
list(APPEND CMAKE_PREFIX_PATH "${PYTHON_EXEC_PREFIX}")
find_package(PythonLibs ${FIND_PYTHON_INTERP_VERSION} MODULE REQUIRED)

find_package(drake CONFIG REQUIRED)

get_filename_component(PYTHONPATH
        "${drake_DIR}/../../python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages"
        REALPATH
        )

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_STANDARD 17)

add_executable(cassie_jump_part1 main.cpp OptNode.cpp OptNode.h Jumping.cpp Jumping.h)

target_link_libraries(cassie_jump_part1 drake::drake ${GFLAGS_LIBRARIES})