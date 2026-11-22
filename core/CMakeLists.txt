# amp_core: the motion planning and control library. No Qt dependency, so every algorithm
# can be unit tested and benchmarked headless.
add_library(amp_core STATIC
    src/arm_model.cpp
    src/kinematics.cpp
    src/geometry.cpp
    src/cspace.cpp
    src/collision.cpp
    src/cspace_grid.cpp
    src/nearest_neighbor.cpp
    src/planner.cpp
    src/rrt.cpp
    src/rrt_star.cpp
    src/astar.cpp
    src/scenario.cpp
    src/smoothing.cpp
    src/trajectory.cpp
    src/dynamics.cpp
    src/pid.cpp
    src/simulation.cpp
)
add_library(amp::core ALIAS amp_core)

target_include_directories(amp_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(amp_core PUBLIC Eigen3::Eigen)
# nlohmann/json is an implementation detail of scenario.cpp. Include it as SYSTEM (rather than
# linking its target) so its headers are exempt from our warnings-as-errors flags.
target_include_directories(amp_core SYSTEM PRIVATE
    $<TARGET_PROPERTY:nlohmann_json::nlohmann_json,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_features(amp_core PUBLIC cxx_std_20)
amp_set_warnings(amp_core)
