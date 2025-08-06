#!/bin/bash
# setup_pico_environment.sh - Complete setup for Pi Pico development environment
# This script installs all required dependencies including USB support

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== Pi Pico Development Environment Setup ===${NC}"

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

# Check if running on Linux
if [[ "$OSTYPE" != "linux-gnu"* ]]; then
    print_error "This script is designed for Linux systems"
    echo "For Windows, use WSL2 or install dependencies manually"
    echo "For macOS, use Homebrew: brew install libusb cmake arm-none-eabi-gcc"
    exit 1
fi

# Update package lists
print_status "Updating package lists..."
sudo apt update

# Install basic development tools
print_status "Installing basic development tools..."
sudo apt install -y \
    build-essential \
    cmake \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    git \
    python3 \
    python3-pip

# Install USB support libraries for picotool
print_status "Installing USB support libraries..."
sudo apt install -y \
    libusb-1.0-0-dev \
    pkg-config \
    libudev-dev

# Install additional useful tools
print_status "Installing additional development tools..."
sudo apt install -y \
    minicom \
    screen \
    gdb-multiarch \
    openocd

# Check if Pico SDK is already installed
if [ -z "$PICO_SDK_PATH" ] || [ ! -d "$PICO_SDK_PATH" ]; then
    print_status "Pico SDK not found. Installing..."
    
    # Create development directory
    mkdir -p ~/pico
    cd ~/pico
    
    # Clone Pico SDK
    if [ ! -d "pico-sdk" ]; then
        git clone https://github.com/raspberrypi/pico-sdk.git
    fi
    
    cd pico-sdk
    git submodule update --init
    
    # Set environment variable
    PICO_SDK_PATH="$HOME/pico/pico-sdk"
    echo "export PICO_SDK_PATH=$PICO_SDK_PATH" >> ~/.bashrc
    
    print_status "Pico SDK installed at: $PICO_SDK_PATH"
    print_warning "Please run 'source ~/.bashrc' or restart your terminal"
else
    print_status "Pico SDK found at: $PICO_SDK_PATH"
fi

# Install picotool for USB support
print_status "Installing picotool with USB support..."
cd ~/pico

if [ ! -d "picotool" ]; then
    git clone https://github.com/raspberrypi/picotool.git
fi

cd picotool
mkdir -p build
cd build

# Configure with USB support
cmake .. -DPICO_SDK_PATH=$PICO_SDK_PATH
make -j$(nproc)

# Install picotool
sudo make install

# Add udev rules for Pico USB access
print_status "Setting up USB permissions for Pico..."
sudo tee /etc/udev/rules.d/99-pico.rules > /dev/null << 'EOF'
# Raspberry Pi Pico
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0003", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0009", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", MODE="0666"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to dialout group for serial port access
print_status "Adding user to dialout group for serial access..."
sudo usermod -a -G dialout $USER

# Test installations
print_status "Testing installations..."
echo "Checking cmake: $(cmake --version | head -1)"
echo "Checking arm-none-eabi-gcc: $(arm-none-eabi-gcc --version | head -1)"
echo "Checking libusb: $(pkg-config --modversion libusb-1.0 2>/dev/null || echo 'Not found')"

if command -v picotool &> /dev/null; then
    echo "Checking picotool: $(picotool version)"
else
    print_warning "picotool not found in PATH. You may need to restart your terminal."
fi

echo ""
echo -e "${GREEN}=== SETUP COMPLETE ===${NC}"
echo ""
echo "Your Pi Pico development environment is ready with USB support!"
echo ""
echo "Important next steps:"
echo "1. Restart your terminal or run: source ~/.bashrc"
echo "2. Log out and log back in (for group membership to take effect)"
echo "3. Test by connecting your Pico and running: picotool info"
echo ""
echo "Environment variables set:"
echo "  PICO_SDK_PATH=$PICO_SDK_PATH"
echo ""
echo "Tools installed:"
echo "  - cmake, arm-none-eabi-gcc (cross-compiler)"
echo "  - libusb-1.0-dev (USB support)"
echo "  - picotool (Pico utilities with USB support)"
echo "  - minicom (serial terminal)"
echo ""
echo "You can now use ./build_pico.sh to build your pendulum code!"

print_status "Setup completed successfully!"
