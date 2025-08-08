cmake_minimum_required(VERSION 3.15)
project(PendulumSimulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# This project no longer depends on SDL2/ImGui.  All user interface
# functionality is provided by the Qt GUI defined below.  As such,
# there is no need to locate SDL2, OpenGL or SDL_ttf packages here.

# Include directories required by the core simulation and control code.
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/embedded")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/adaptive_mass")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/common")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/qt_gui")

#---------------------------------------------------------------------
# Qt GUI
#
# We build the Qt‑based GUI by default.  This retained‑mode interface
# replaces the previous SDL/ImGui implementation entirely.  The Qt
# option is enabled by default so the simulator always compiles with
# the new interface.  You can set BUILD_QT_GUI to OFF to disable
# building the GUI entirely (e.g. when building for embedded only).

option(BUILD_QT_GUI "Build Qt GUI" ON)
if(BUILD_QT_GUI)
    # Find Qt6 first, fall back to Qt5 if necessary
    find_package(Qt6 COMPONENTS Widgets OpenGL QUIET)
    if(Qt6_FOUND)
        set(QT_LIBS Qt6::Widgets Qt6::OpenGL)
        set(QT_WRAP qt6_wrap_cpp)
    else()
        find_package(Qt5 COMPONENTS Widgets OpenGL REQUIRED)
        set(QT_LIBS Qt5::Widgets Qt5::OpenGL)
        set(QT_WRAP qt5_wrap_cpp)
    endif()

    # List of header files requiring moc generation
    set(QT_MOC_HEADERS
        qt_gui/ControlPanel.h
        qt_gui/MainWindow.h
        qt_gui/PendulumWidget.h
    )
    ${QT_WRAP}(MOC_SRCS ${QT_MOC_HEADERS})

    # Sources for the Qt application.  We reuse simulation and control
    # modules from the embedded implementation to ensure consistent
    # behaviour across platforms.
    set(QT_APP_SOURCES
        qt_gui/qt_main.cpp
        qt_gui/PendulumWidget.cpp
        qt_gui/ControlPanel.cpp
        qt_gui/MainWindow.cpp
        enhanced_physics_simulation.cpp
        unified_virtual_encoder.c
        embedded/motor_protection.c
        embedded/drv8833.c
        embedded/pwm_simulation.c
        adaptive_mass/mass_friction_estimator.c
        adaptive_mass/mass_integration.c
        common/energy_control.c
        ${MOC_SRCS}
    )
    # Build the Qt GUI as the primary simulator executable
    add_executable(${PROJECT_NAME}
        ${QT_APP_SOURCES}
    )
    target_compile_definitions(${PROJECT_NAME} PRIVATE QT_GUI)
    target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/qt_gui
        ${CMAKE_CURRENT_SOURCE_DIR}/embedded
        ${CMAKE_CURRENT_SOURCE_DIR}/adaptive_mass
        ${CMAKE_CURRENT_SOURCE_DIR}/common
    )
    target_link_libraries(${PROJECT_NAME} PRIVATE ${QT_LIBS} ${CMAKE_DL_LIBS})
    install(TARGETS ${PROJECT_NAME} DESTINATION bin)
endif()