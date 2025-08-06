#!/bin/bash
# build_pico.sh - Build script for Pi Pico deployment
# This script builds the same pendulum_simulator.cpp for Pico hardware
# Usage: ./build_pico.sh [--clean]

set -e  # Exit on any error

# Parse command line arguments
CLEAN_BUILD=false
if [[ "$1" == "--clean" ]]; then
    CLEAN_BUILD=true
    echo "Clean build requested"
fi

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== Pi Pico Build Script for Pendulum Control ===${NC}"

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if PICO_SDK_PATH is set
if [ -z "$PICO_SDK_PATH" ]; then
    print_error "PICO_SDK_PATH environment variable is not set!"
    echo "Please install Pico SDK and set PICO_SDK_PATH:"
    echo "  git clone https://github.com/raspberrypi/pico-sdk.git"
    echo "  export PICO_SDK_PATH=/path/to/pico-sdk"
    echo "  git submodule update --init"
    exit 1
fi

print_status "Using Pico SDK at: $PICO_SDK_PATH"

# Check if required tools are installed
check_tool() {
    if ! command -v "$1" &> /dev/null; then
        print_error "$1 is not installed!"
        echo "Install with: sudo apt install $2"
        exit 1
    fi
}

print_status "Checking build tools..."
check_tool "cmake" "cmake"
check_tool "arm-none-eabi-gcc" "gcc-arm-none-eabi"
check_tool "make" "build-essential"

# Check for USB support libraries and install if missing
print_status "Checking USB support libraries..."
if ! pkg-config --exists libusb-1.0; then
    print_status "Installing USB support libraries..."
    sudo apt update && sudo apt install -y libusb-1.0-0-dev pkg-config
    
    # Verify installation
    if pkg-config --exists libusb-1.0; then
        print_status "USB support libraries installed successfully"
    else
        print_warning "Failed to install USB libraries - continuing anyway"
    fi
else
    print_status "USB support libraries found"
fi

# Check if source file exists
if [ ! -f "pendulum_simulator.cpp" ]; then
    print_error "pendulum_simulator.cpp not found in current directory!"
    exit 1
fi

# Check if PC_DEBUG is defined (warn user)
if grep -q "^#define PC_DEBUG" pendulum_simulator.cpp; then
    print_warning "PC_DEBUG is still defined in pendulum_simulator.cpp"
    print_warning "This will cause compilation errors for Pico!"
    echo "Please comment out or remove the '#define PC_DEBUG' line"
    echo "Would you like me to do this automatically? (y/N)"
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        sed -i.bak 's/^#define PC_DEBUG/\/\/ #define PC_DEBUG/' pendulum_simulator.cpp
        print_status "Commented out PC_DEBUG definition (backup saved as .bak)"
    else
        print_error "Please fix PC_DEBUG definition and try again"
        exit 1
    fi
fi

# Create pico_sdk_import.cmake if it doesn't exist
if [ ! -f "pico_sdk_import.cmake" ]; then
    print_status "Copying pico_sdk_import.cmake..."
    cp "$PICO_SDK_PATH/external/pico_sdk_import.cmake" .
fi

# Create Pico-specific CMakeLists.txt
print_status "Creating Pico CMakeLists.txt..."
cat > CMakeLists_pico.txt << 'EOF'
cmake_minimum_required(VERSION 3.13)

# Include Pico SDK
include(pico_sdk_import.cmake)

project(pendulum_control)

# Initialize the SDK
pico_sdk_init()

# Add executable with all source files
add_executable(pendulum_control
    embedded/main.c
    embedded/as5600.c
    embedded/control.c
    embedded/debug.c
    embedded/drv8833.c
    embedded/filters.c
    embedded/motor_protection.c
    embedded/pwm_simulation.c
    embedded/virtual_encoder.c
)

# Include directories
target_include_directories(pendulum_control PRIVATE embedded/)

# Link Pico libraries
target_link_libraries(pendulum_control 
    pico_stdlib 
    hardware_pwm 
    hardware_i2c 
    hardware_gpio
    hardware_timer
    hardware_watchdog
)

# Enable USB serial for debugging
pico_enable_stdio_usb(pendulum_control 1)
pico_enable_stdio_uart(pendulum_control 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(pendulum_control)

# Add compile definitions for Pico build
target_compile_definitions(pendulum_control PRIVATE
    PLATFORM_PICO=1
)
EOF

# Create build directory
BUILD_DIR="build_pico"
if [ "$CLEAN_BUILD" = true ]; then
    if [ -d "$BUILD_DIR" ]; then
        print_status "Clean build: removing existing build directory..."
        rm -rf "$BUILD_DIR"
    fi
    print_status "Creating fresh build directory: $BUILD_DIR"
    mkdir "$BUILD_DIR"
elif [ ! -d "$BUILD_DIR" ]; then
    print_status "Creating build directory: $BUILD_DIR"
    mkdir "$BUILD_DIR"
else
    print_status "Using existing build directory: $BUILD_DIR (incremental build)"
fi

# Copy/update required files to build directory
print_status "Updating source files..."
cp pendulum_simulator.cpp "$BUILD_DIR/"
cp CMakeLists_pico.txt "$BUILD_DIR/CMakeLists.txt"
cp pico_sdk_import.cmake "$BUILD_DIR/"

# Copy embedded directory with all source files (force update)
print_status "Updating embedded source files..."
if [ -d "$BUILD_DIR/embedded" ]; then
    rm -rf "$BUILD_DIR/embedded"
fi
cp -r embedded "$BUILD_DIR/"

# Enter build directory
cd "$BUILD_DIR"

# Configure with CMake for Pico
print_status "Configuring build with CMake for Pico..."
cmake -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico .

# Build the project
print_status "Building for Pi Pico..."
make -j$(nproc)

# Check if build was successful
if [ -f "pendulum_control.uf2" ]; then
    print_status "Build successful!"
    echo
    echo -e "${GREEN}=== BUILD COMPLETE ===${NC}"
    echo "Generated files:"
    echo "  - pendulum_control.uf2 (drag to Pico in bootloader mode)"
    echo "  - pendulum_control.elf (for debugging)"
    echo "  - pendulum_control.bin (raw binary)"
    echo
    echo "To upload to Pico:"
    echo "1. Hold BOOTSEL button while connecting Pico to USB"
    echo "2. Drag build_pico/pendulum_control.uf2 to the Pico drive"
    echo "3. Pico will reboot and run the pendulum control code"
    echo
    echo "Monitor output with: minicom -D /dev/ttyACM0 -b 115200"
    echo
    echo "For troubleshooting:"
    echo "  - Use './build_pico.sh --clean' to force a clean rebuild"
    echo "  - Check that PICO_SDK_PATH is set correctly"
else
    print_error "Build failed! Check error messages above."
    exit 1
fi

# Return to original directory
cd ..

print_status "Pico build complete!"
