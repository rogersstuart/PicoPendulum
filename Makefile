# Makefile for Pendulum Simulator
# Alternative to CMake for simpler builds

# Compiler settings
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -DPC_SIMULATION
INCLUDES = -Iexternal/imgui -Iexternal/imgui/backends -Iexternal/gl3w/include -Iembedded
LIBS = -lSDL2 -lGL -ldl -lpthread

# Platform detection
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S), Linux)
    LIBS += -lGL -lX11
endif
ifeq ($(UNAME_S), Darwin)
    LIBS = -lSDL2 -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo
endif

# Directories
BUILD_DIR = build
EXTERNAL_DIR = external
SRC_DIR = .

# Source files
IMGUI_SOURCES = $(EXTERNAL_DIR)/imgui/imgui.cpp \
                $(EXTERNAL_DIR)/imgui/imgui_demo.cpp \
                $(EXTERNAL_DIR)/imgui/imgui_draw.cpp \
                $(EXTERNAL_DIR)/imgui/imgui_tables.cpp \
                $(EXTERNAL_DIR)/imgui/imgui_widgets.cpp \
                $(EXTERNAL_DIR)/imgui/backends/imgui_impl_sdl2.cpp \
                $(EXTERNAL_DIR)/imgui/backends/imgui_impl_opengl3.cpp

GL3W_SOURCES = $(EXTERNAL_DIR)/gl3w/src/gl3w.c

APP_SOURCES = pendulum_simulator.cpp

ALL_SOURCES = $(APP_SOURCES) $(IMGUI_SOURCES) $(GL3W_SOURCES)

# Object files
OBJECTS = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(patsubst %.c,$(BUILD_DIR)/%.o,$(ALL_SOURCES)))

# Target executable
TARGET = $(BUILD_DIR)/PendulumSimulator

# Default target
all: setup $(TARGET)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)
	mkdir -p $(BUILD_DIR)/external/imgui
	mkdir -p $(BUILD_DIR)/external/imgui/backends
	mkdir -p $(BUILD_DIR)/external/gl3w/src

# Setup external dependencies
setup: $(EXTERNAL_DIR)/imgui $(EXTERNAL_DIR)/gl3w

$(EXTERNAL_DIR)/imgui:
	@echo "Setting up ImGui..."
	mkdir -p $(EXTERNAL_DIR)
	cd $(EXTERNAL_DIR) && git clone https://github.com/ocornut/imgui.git

$(EXTERNAL_DIR)/gl3w:
	@echo "Setting up GL3W..."
	mkdir -p $(EXTERNAL_DIR)
	cd $(EXTERNAL_DIR) && git clone https://github.com/skaslev/gl3w.git
	cd $(EXTERNAL_DIR)/gl3w && python3 gl3w_gen.py

# Build target
$(TARGET): $(BUILD_DIR) $(OBJECTS)
	@echo "Linking $(TARGET)..."
	$(CXX) $(OBJECTS) -o $(TARGET) $(LIBS)
	@echo "Build complete!"

# Compile C++ sources
$(BUILD_DIR)/%.o: %.cpp
	@echo "Compiling $<..."
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Compile C sources
$(BUILD_DIR)/%.o: %.c
	@echo "Compiling $<..."
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Run the simulator
run: $(TARGET)
	./$(TARGET)

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR)

# Clean everything including external dependencies
distclean: clean
	rm -rf $(EXTERNAL_DIR)

# Install system dependencies (Ubuntu/Debian)
deps-ubuntu:
	sudo apt-get update
	sudo apt-get install -y build-essential libsdl2-dev libgl1-mesa-dev python3 git

# Install system dependencies (macOS with Homebrew)
deps-macos:
	brew install sdl2 python3 git

# Print help
help:
	@echo "Pendulum Simulator Makefile"
	@echo "Usage:"
	@echo "  make [all]     - Build the simulator (default)"
	@echo "  make setup     - Setup external dependencies"
	@echo "  make run       - Build and run the simulator"
	@echo "  make clean     - Clean build artifacts"
	@echo "  make distclean - Clean everything including external deps"
	@echo "  make deps-ubuntu - Install dependencies on Ubuntu/Debian"
	@echo "  make deps-macos  - Install dependencies on macOS"
	@echo "  make help      - Show this help"

# Declare phony targets
.PHONY: all setup run clean distclean deps-ubuntu deps-macos help