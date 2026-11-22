# Qt 6 Widgets GUI. A thin client of amp_core: all algorithms live in the core library.
add_executable(amp_app
    main.cpp
    main_window.cpp
    main_window.hpp
    scene_model.cpp
    scene_model.hpp
    workspace_view.cpp
    workspace_view.hpp
    cspace_view.cpp
    cspace_view.hpp
    plot_widget.cpp
    plot_widget.hpp
    controls_panel.cpp
    controls_panel.hpp
    planning_controller.cpp
    planning_controller.hpp
    scenario_paths.hpp
    view_style.hpp
)

set_target_properties(amp_app PROPERTIES
    OUTPUT_NAME arm-motion-planner
    AUTOMOC ON
    WIN32_EXECUTABLE ON)

target_link_libraries(amp_app PRIVATE amp::core Qt6::Widgets)
amp_set_warnings(amp_app)
target_compile_definitions(amp_app PRIVATE AMP_SCENARIO_DIR="${PROJECT_SOURCE_DIR}/scenarios")

# Ship the scenario files next to the executable so a deployed build is self-contained.
add_custom_command(TARGET amp_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/scenarios $<TARGET_FILE_DIR:amp_app>/scenarios
    COMMENT "Copying scenarios next to the application")

if(WIN32)
    # Copy the Qt runtime (DLLs, platform plugin, compiler runtime) next to the executable
    # after every build so it can be launched directly without Qt on PATH.
    get_target_property(_qmake_exe Qt6::qmake IMPORTED_LOCATION)
    get_filename_component(_qt_bin_dir "${_qmake_exe}" DIRECTORY)
    find_program(WINDEPLOYQT_EXECUTABLE windeployqt HINTS "${_qt_bin_dir}")
    if(WINDEPLOYQT_EXECUTABLE)
        add_custom_command(TARGET amp_app POST_BUILD
            COMMAND "${WINDEPLOYQT_EXECUTABLE}" --no-translations --no-system-d3d-compiler
                    --no-opengl-sw "$<TARGET_FILE:amp_app>"
            COMMENT "Deploying Qt runtime with windeployqt"
            VERBATIM)
    else()
        message(WARNING "windeployqt not found; Qt DLLs will not be deployed next to the app.")
    endif()
endif()
