include(GoogleTest)

add_executable(amp_tests
    test_kinematics.cpp
    test_geometry.cpp
    test_collision.cpp
    test_nearest_neighbor.cpp
    test_planners.cpp
    test_scenario.cpp
    test_postprocessing.cpp
    test_control.cpp
)
target_compile_definitions(amp_tests PRIVATE AMP_SCENARIO_DIR="${PROJECT_SOURCE_DIR}/scenarios")
target_link_libraries(amp_tests PRIVATE amp::core GTest::gtest_main)
amp_set_warnings(amp_tests)

# DISCOVERY_MODE PRE_TEST: discover tests when ctest runs, not at build time, so a
# missing runtime DLL never breaks the build itself.
gtest_discover_tests(amp_tests DISCOVERY_MODE PRE_TEST)

# Traceability gate: fails if any requirement in docs/REQUIREMENTS.md has no test that
# declares "@verifies REQ-..." for it, or if a test references an unknown requirement.
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_FOUND)
    add_test(NAME traceability_check
        COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tools/check_traceability.py --check
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
else()
    message(WARNING "Python 3 not found: the traceability_check test will not be registered.")
endif()
