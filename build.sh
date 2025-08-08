#!/bin/bash
# Legacy build script for the pendulum simulator.
#
# This script previously handled building the SDL/ImGui version of the
# simulator.  The project now uses a Qt‑based GUI and is built with
# CMake.  This script is retained for reference and may not be
# required for normal use.

echo "=== Pendulum Control Simulator Build Script ==="

# Check if we're on the right platform
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    PLATFORM="Linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    PLATFORM="macOS"
elif [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "win32" ]]; then
    PLATFORM="Windows"
else
    echo "Unknown platform: $OSTYPE"
    exit 1
fi

echo "Building for: $PLATFORM"

# Create directory structure
echo "Creating project structure..."
mkdir -p build
mkdir -p external
mkdir -p embedded

# Function to download and extract ImGui
setup_imgui() {
    echo "Setting up ImGui..."
    if [ ! -d "external/imgui" ]; then
        cd external
        echo "Downloading ImGui..."
        git clone https://github.com/ocornut/imgui.git
        cd ..
    else
        echo "ImGui already exists, updating..."
        cd external/imgui
        git pull
        cd ../..
    fi
}

# Function to download and extract GL3W
setup_gl3w() {
    echo "Setting up GL3W..."
    if [ ! -d "external/gl3w" ]; then
        cd external
        echo "Downloading GL3W..."
        git clone https://github.com/skaslev/gl3w.git
        cd gl3w
        python3 gl3w_gen.py
        cd ../..
    else
        echo "GL3W already exists"
    fi
}

# Function to install dependencies based on platform
install_dependencies() {
    echo "Installing dependencies for $PLATFORM..."
    
    case $PLATFORM in
        "Linux")
            # Ubuntu/Debian
            if command -v apt-get >/dev/null 2>&1; then
                sudo apt-get update
                sudo apt-get install -y cmake build-essential libsdl2-dev libsdl2-ttf-dev libgl1-mesa-dev libglu1-mesa-dev python3
            # Fedora/RHEL
            elif command -v dnf >/dev/null 2>&1; then
                sudo dnf install -y cmake gcc-c++ SDL2-devel SDL2_ttf-devel mesa-libGL-devel mesa-libGLU-devel python3
            # Arch Linux
            elif command -v pacman >/dev/null 2>&1; then
                sudo pacman -S cmake gcc sdl2 sdl2_ttf mesa glu python
            else
                echo "Please install: cmake, gcc/g++, SDL2 development libraries, SDL2_ttf development libraries, OpenGL development libraries, python3"
            fi
            ;;
        "macOS")
            # Check if Homebrew is installed
            if command -v brew >/dev/null 2>&1; then
                brew install cmake sdl2 python3
            else
                echo "Please install Homebrew and run: brew install cmake sdl2 python3"
                echo "Or install dependencies manually"
            fi
            ;;
        "Windows")
            echo "For Windows, please ensure you have:"
            echo "1. Visual Studio 2019 or later with C++ support"
            echo "2. CMake"
            echo "3. vcpkg package manager"
            echo "Then run: vcpkg install sdl2 opengl"
            ;;
    esac
}

# Function to build the project
build_project() {
    echo "Building project..."
    
    # Setup external dependencies
    setup_imgui
    setup_gl3w
    
    # Create build directory and run CMake
    cd build
    
    case $PLATFORM in
        "Windows")
            cmake -G "Visual Studio 16 2019" -A x64 ..
            cmake --build . --config Release
            ;;
        *)
            cmake -DCMAKE_BUILD_TYPE=Release ..
            make -j$(nproc)
            ;;
    esac
    
    cd ..
    
    if [ $? -eq 0 ]; then
        echo "Build successful!"
        echo "Executable location:"
        case $PLATFORM in
            "Windows")
                echo "  build/Release/PendulumSimulator.exe"
                ;;
            *)
                echo "  build/PendulumSimulator"
                ;;
        esac
    else
        echo "Build failed!"
        exit 1
    fi
}

# Function to run the simulator
run_simulator() {
    echo "Running Pendulum Simulator..."
    case $PLATFORM in
        "Windows")
            if [ -f "build/Release/PendulumSimulator.exe" ]; then
                ./build/Release/PendulumSimulator.exe
            else
                echo "Executable not found. Please build first."
                exit 1
            fi
            ;;
        *)
            if [ -f "build/PendulumSimulator" ]; then
                ./build/PendulumSimulator
            else
                echo "Executable not found. Please build first."
                exit 1
            fi
            ;;
    esac
}

# Main script logic
case ${1:-build} in
    "deps")
        install_dependencies
        ;;
    "setup")
        install_dependencies
        setup_imgui
        setup_gl3w
        ;;
    "build")
        build_project
        ;;
    "run")
        run_simulator
        ;;
    "clean")
        echo "Cleaning build directory..."
        rm -rf build
        echo "Clean complete."
        ;;
    "help")
        echo "Usage: $0 [command]"
        echo "Commands:"
        echo "  deps  - Install system dependencies"
        echo "  setup - Setup external libraries (ImGui, GL3W)"
        echo "  build - Build the project (default)"
        echo "  run   - Run the simulator"
        echo "  clean - Clean build directory"
        echo "  help  - Show this help"
        ;;
    *)
        echo "Unknown command: $1"
        echo "Use '$0 help' for usage information"
        exit 1
        ;;
esac