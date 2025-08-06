cmake_minimum_required(VERSION 3.15)
project(PendulumSimulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(SDL2 REQUIRED)
find_package(OpenGL REQUIRED)

# Try to find SDL2_ttf
find_package(SDL2_ttf QUIET)
if(NOT SDL2_ttf_FOUND)
    # Fallback to pkg-config
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(SDL2_TTF SDL2_ttf)
    endif()
endif()

# Include directories
include_directories(${SDL2_INCLUDE_DIRS})
if(SDL2_ttf_FOUND)
    include_directories(${SDL2_TTF_INCLUDE_DIRS})
elseif(SDL2_TTF_FOUND)
    include_directories(${SDL2_TTF_INCLUDE_DIRS})
endif()
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/external/imgui")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/external/imgui/backends")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/external/gl3w/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/embedded")

# ImGui sources
set(IMGUI_SOURCES
    external/imgui/imgui.cpp
    external/imgui/imgui_demo.cpp
    external/imgui/imgui_draw.cpp
    external/imgui/imgui_tables.cpp
    external/imgui/imgui_widgets.cpp
    external/imgui/backends/imgui_impl_sdl2.cpp
    external/imgui/backends/imgui_impl_opengl3.cpp
)

# GL3W sources (alternative: use GLEW)
set(GL3W_SOURCES
    external/gl3w/src/gl3w.c
)

# Main application source
set(APP_SOURCES
    pendulum_simulator.cpp
    enhanced_physics_simulation.cpp
    embedded/virtual_encoder.c
    embedded/motor_protection.c
    embedded/drv8833.c
    embedded/pwm_simulation.c
)

# Create the executable
add_executable(${PROJECT_NAME} 
    ${APP_SOURCES}
    ${IMGUI_SOURCES} 
    ${GL3W_SOURCES}
)

# Define PC_DEBUG for conditional compilation
target_compile_definitions(${PROJECT_NAME} PRIVATE PC_DEBUG)

# Link libraries
set(LINK_LIBRARIES
    ${SDL2_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${CMAKE_DL_LIBS}
)

# Add SDL2_ttf if found
if(SDL2_ttf_FOUND)
    list(APPEND LINK_LIBRARIES ${SDL2_TTF_LIBRARIES})
elseif(SDL2_TTF_FOUND)
    list(APPEND LINK_LIBRARIES ${SDL2_TTF_LIBRARIES})
endif()

target_link_libraries(${PROJECT_NAME} ${LINK_LIBRARIES})

# Compiler-specific options
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic)
endif()

# Copy assets or data files if needed
# file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/assets DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# For debugging
set_target_properties(${PROJECT_NAME} PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)

# Platform-specific settings
if(WIN32)
    target_compile_definitions(${PROJECT_NAME} PRIVATE WIN32_LEAN_AND_MEAN)
    target_link_libraries(${PROJECT_NAME} opengl32 gdi32)
elseif(APPLE)
    target_link_libraries(${PROJECT_NAME} "-framework OpenGL" "-framework Cocoa" "-framework IOKit" "-framework CoreVideo")
else()
    target_link_libraries(${PROJECT_NAME} GL X11 pthread dl)
endif()

# Installation rules
install(TARGETS ${PROJECT_NAME} DESTINATION bin)