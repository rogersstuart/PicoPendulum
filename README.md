# Pendulum Control Simulator

A PC application that wraps your embedded pendulum control code with a realistic physics simulator. This allows you to test and tune your control algorithms without hardware.

## Features

- **Realistic Physics Simulation**: RK4 integration with proper pendulum dynamics
- **Complete Control System**: Your exact embedded control algorithms
- **Real-time Visualization**: OpenGL rendering of pendulum motion
- **Interactive GUI**: Parameter tuning, data logging, and control
- **Data Export**: CSV export for analysis in Excel/MATLAB/Python
- **Noise Simulation**: Configurable measurement and process noise
- **Cross-platform**: Linux, macOS, and Windows support

## Prerequisites

### Linux (Ubuntu/Debian)
```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libsdl2-dev libgl1-mesa-dev python3 git
```

### Linux (Fedora/RHEL)
```bash
sudo dnf install -y cmake gcc-c++ SDL2-devel mesa-libGL-devel python3 git
```

### macOS
```bash
# Install Homebrew first: https://brew.sh
brew install cmake sdl2 python3 git
```

### Windows
1. Install Visual Studio 2019 or later with C++ support
2. Install CMake
3. Install vcpkg package manager
4. Run: `vcpkg install sdl2 opengl`

## Quick Start

### Method 1: Using the Build Script (Recommended)
```bash
# Clone and setup
git clone <your-repo-url>
cd pendulum-simulator

# Make build script executable
chmod +x build.sh

# Install dependencies
./build.sh deps

# Setup external libraries and build
./build.sh setup
./build.sh build

# Run the simulator
./build.sh run
```

### Method 2: Using CMake
```bash
# Setup external dependencies
mkdir external && cd external
git clone https://github.com/ocornut/imgui.git
git clone https://github.com/skaslev/gl3w.git
cd gl3w && python3 gl3w_gen.py && cd ../..

# Build
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# Run
./PendulumSimulator
```

### Method 3: Using Makefile
```bash
# Install dependencies (Ubuntu)
make deps-ubuntu

# Build and run
make run
```

## Project Structure

```
pendulum-simulator/
├── pendulum_simulator.cpp     # Main PC application
├── CMakeLists.txt             # CMake build system
├── Makefile                   # Alternative build system
├── build.sh                   # Build script
├── embedded/                  # Modified embedded headers
│   ├── control.h             # Control system header
│   ├── drv8833.h             # Motor driver header (mocked)
│   └── filters.h             # Filters and Kalman filter
├── external/                  # External dependencies (auto-downloaded)
│   ├── imgui/                # Dear ImGui library
│   └── gl3w/                 # OpenGL loader
└── build/                    # Build artifacts
```

## Usage

### Controls
- **Space**: Pause/Resume simulation
- **S**: Single step when paused
- **R**: Reset simulation
- **1**: Set control state to IDLE
- **2**: Set control state to SWING-UP

### GUI Panels

#### Control Panel
- **Simulation Control**: Pause, step, reset, time scaling
- **State Display**: Real-time pendulum state and energy
- **Control Parameters**: Live tuning of PID gains and energy control
- **Physics Parameters**: Adjust mass, length, damping, friction, noise
- **Control Actions**: Start swing-up, apply disturbances

#### Data Logging Panel
- **Enable/Disable Logging**: Toggle data collection
- **Real-time Plots**: Angle, energy, and motor command visualization
- **CSV Export**: Save data for external analysis

### Testing Your Control Algorithm

1. **Start with Default Parameters**: The simulator loads realistic parameters matching your embedded system
2. **Test Swing-up**: Click "Start Swing-up" or press '2' to test energy pumping
3. **Apply Disturbances**: Use "Small Kick" or "Large Kick" to test robustness
4. **Tune Parameters**: Adjust gains in real-time and observe the response
5. **Add Noise**: Enable measurement/process noise to test filtering
6. **Export Data**: Save CSV files for detailed analysis

### Parameter Tuning Workflow

1. **Start Conservative**: Begin with low gains (Kp=5, Kd=1, Ki=0.1)
2. **Increase Proportional**: Raise Kp until you get good tracking
3. **Add Derivative**: Increase Kd to reduce oscillations
4. **Fine-tune Integral**: Add Ki slowly to eliminate steady-state error
5. **Test Robustness**: Apply disturbances and add noise
6. **Validate Energy Control**: Ensure swing-up works reliably

## Code Integration

The simulator uses preprocessing directives to adapt your embedded code:

```c
#ifdef PC_SIMULATION
    // PC-specific implementations
    typedef float float32_t;
    // Mock hardware functions
#else
    // Original embedded code
    #include "pico/stdlib.h"
    // Real hardware functions
#endif
```

This means:
- **No changes needed** to your core control algorithms
- **Same exact code** runs on both PC and embedded
- **Easy validation** that PC results match hardware

## Physics Model

The simulator implements a realistic pendulum model:

- **Equation of Motion**: `I*α = -mgl*sin(θ) - b*ω - f*sgn(ω) + τ`
- **Integration**: 4th-order Runge-Kutta for accuracy
- **Noise**: Configurable measurement and process noise
- **Friction**: Both viscous damping and Coulomb friction

Where:
- `I` = moment of inertia (rod about pivot)
- `m` = pendulum mass
- `l` = distance to center of mass
- `g` = gravity (9.81 m/s²)
- `b` = viscous damping coefficient
- `f` = Coulomb friction
- `τ` = applied torque from motor

## Troubleshooting

### Build Issues
- **Missing SDL2**: Install development packages (`libsdl2-dev` on Ubuntu)
- **OpenGL errors**: Install Mesa development packages
- **Python3 not found**: Install Python 3 for GL3W generation

### Runtime Issues
- **Blank window**: Check OpenGL drivers are installed
- **Poor performance**: Try reducing time scale or window size
- **Unstable simulation**: Reduce time step or check for NaN values

### Control Issues
- **Won't swing up**: Check energy target and motor limits
- **Oscillates in balance**: Reduce gains or increase damping
- **Falls immediately**: Check sign conventions and initial conditions

## Advanced Usage

### Custom Physics Parameters
Modify the `PendulumPhysics` class to match your specific hardware:
```cpp
physics.setParameters(
    0.2f,    // mass (kg)
    0.3048f, // length (m) 
    0.001f,  // damping
    0.002f   // friction
);
```

### Data Analysis
Export CSV files and analyze in Python:
```python
import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv('pendulum_data.csv')
plt.plot(data['time'], data['theta_u'])
plt.xlabel('Time (s)')
plt.ylabel('Angle from Upright (rad)')
plt.show()
```

### Performance Optimization
- Reduce visualization rate for faster simulation
- Use release build for maximum performance
- Adjust integration time step for accuracy vs speed tradeoff

## Contributing

1. Fork the repository
2. Create a feature branch
3. Test with both simulation and hardware
4. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- **Dear ImGui**: Excellent immediate mode GUI library
- **SDL2**: Cross-platform windowing and input
- **Your embedded code**: The excellent control algorithms being simulated!