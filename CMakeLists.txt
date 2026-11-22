cmake_minimum_required(VERSION 3.21)

project(arm-motion-planner
    VERSION 0.1.0
    DESCRIPTION "Planar robot arm motion planning and control demo (RRT, RRT*, A*, PID)"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Put all executables in one folder so tests, benchmark and app are easy to find.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(AMP_BUILD_TESTS "Build the GoogleTest unit tests" ON)
option(AMP_BUILD_BENCH "Build the planner benchmark executable" ON)
option(AMP_BUILD_APP   "Build the Qt 6 Widgets application (requires Qt 6)" ON)

include(cmake/CompilerWarnings.cmake)
include(cmake/Dependencies.cmake)

add_subdirectory(core)

if(AMP_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(AMP_BUILD_BENCH AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/bench/CMakeLists.txt)
    add_subdirectory(bench)
endif()

if(AMP_BUILD_APP AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/app/CMakeLists.txt)
    # The GUI is optional: the core library and tests must build on machines without Qt.
    find_package(Qt6 COMPONENTS Widgets QUIET)
    if(Qt6_FOUND)
        add_subdirectory(app)
    else()
        message(WARNING "Qt 6 Widgets not found: skipping the GUI application. "
                        "Set CMAKE_PREFIX_PATH to your Qt kit (e.g. C:/Qt/6.9.2/msvc2022_64) to build it.")
    endif()
endif()
