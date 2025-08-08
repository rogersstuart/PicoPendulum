#!/bin/bash
# Complete build script for the pendulum simulator with Qt GUI.
#
# This script handles everything from dependency installation to running the app.
# Run with: ./build.sh

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

# Function to install dependencies based on platform
install_dependencies() {
    echo "Installing dependencies for $PLATFORM..."
    
    case $PLATFORM in
        "Linux")
            # Ubuntu/Debian
            if command -v apt-get >/dev/null 2>&1; then
                echo "Installing packages with apt-get..."
                sudo apt-get update
                sudo apt-get install -y cmake build-essential qtbase5-dev libqt5opengl5-dev qt5-qmake
            # Fedora/RHEL
            elif command -v dnf >/dev/null 2>&1; then
                echo "Installing packages with dnf..."
                sudo dnf install -y cmake gcc-c++ qt5-qtbase-devel qt5-devel
            # Arch Linux
            elif command -v pacman >/dev/null 2>&1; then
                echo "Installing packages with pacman..."
                sudo pacman -S cmake gcc qt5-base qt5-tools
            else
                echo "Unsupported Linux distribution. Please install manually:"
                echo "- cmake"
                echo "- gcc/g++"
                echo "- Qt5 development libraries (qtbase5-dev, libqt5opengl5-dev)"
                exit 1
            fi
            ;;
        "macOS")
            # Check if Homebrew is installed
            if command -v brew >/dev/null 2>&1; then
                echo "Installing packages with Homebrew..."
                brew install cmake qt5
                export PATH="/usr/local/opt/qt5/bin:$PATH"
            else
                echo "Please install Homebrew first: https://brew.sh"
                echo "Then run this script again"
                exit 1
            fi
            ;;
        "Windows")
            echo "For Windows/WSL, please ensure you have:"
            echo "1. CMake"
            echo "2. Qt5 development libraries"
            echo "Run: sudo apt-get install cmake build-essential qtbase5-dev libqt5opengl5-dev"
            ;;
    esac
}

# Function to configure and build the project
build_project() {
    echo "Configuring and building project..."
    
    # Create build directory
    mkdir -p build
    cd build
    
    # Configure with CMake
    echo "Running CMake configuration..."
    cmake -DCMAKE_BUILD_TYPE=Release ..
    
    if [ $? -ne 0 ]; then
        echo "CMake configuration failed!"
        echo "Make sure Qt5 is properly installed."
        cd ..
        exit 1
    fi
    
    # Build the project
    echo "Building project..."
    make -j$(nproc 2>/dev/null || echo 4)
    
    cd ..
    
    if [ $? -eq 0 ]; then
        echo "Build successful!"
        echo "Executable: build/PendulumSimulator"
    else
        echo "Build failed!"
        exit 1
    fi
}

# Function to run the simulator
run_simulator() {
    echo "Running Pendulum Simulator..."
    if [ -f "build/PendulumSimulator" ]; then
        ./build/PendulumSimulator
    else
        echo "Executable not found. Building first..."
        build_project
        if [ -f "build/PendulumSimulator" ]; then
            ./build/PendulumSimulator
        else
            echo "Failed to build executable"
            exit 1
        fi
    fi
}

# Function to clean build directory
clean_build() {
    echo "Cleaning build directory..."
    rm -rf build
    echo "Clean complete."
}

# Main execution logic
echo ""
echo "Step 1: Installing dependencies..."
install_dependencies

echo ""
echo "Step 2: Building project..."
build_project

echo ""
echo "Step 3: Starting application..."
run_simulator
