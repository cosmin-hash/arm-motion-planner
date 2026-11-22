# Planner and nearest-neighbour benchmark. Prints Markdown tables for the README.
add_executable(amp_bench amp_bench.cpp)
target_link_libraries(amp_bench PRIVATE amp::core)
target_compile_definitions(amp_bench PRIVATE AMP_SCENARIO_DIR="${PROJECT_SOURCE_DIR}/scenarios")
amp_set_warnings(amp_bench)
