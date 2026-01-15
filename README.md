# Adamant-Xmera Flight Software Algorithms
This repository contains the GNC FSW algorithms.
asasdasda
## Getting Started
This repository has two primary usages and therefore two build paths:
1. To compile the FSW algorithms as modules for the Xmera simulation tool.
2. To compile the FSW algorithms as a single library for use within the Adamant system.

The build Xmera build path is controlled by the Xmera build system. It has the directory structure:
- algorithms/
  - CMakeLists.txt
  - alg1/
    - CMakeLists.txt
    - module.i
    - module.h
    - module.cpp
  - alg2/
    - etc.

The flight software build path is controlled by the CMake build system.

### Dependencies:
- Eigen for freestanding C++ (it's assumed that the Eigen repo is at ../ relative to this repo)

### Required Tooling:
- GCC 13.3.1 or higher
- CMake 3.24 or higher
- Python 3.10 or higher

### Linux
1. Install GCC 13.3.1 with `sudo apt install gcc-13`
2. Test GCC with `gcc --version` - this command should output information about the installed GCC compiler.

If you want to set GCC 13 as your default compiler;
```
# Add the compiler versions to the alternatives system
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-X.Y PRIORITY
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-X.Y PRIORITY
# Replace X.Y with version number and PRIORITY with a numerical value (higher is preferred e.g. 100)

# Configure the default compiler interactively
sudo update-alternatives --config gcc
sudo update-alternatives --config g++
```

### Mac OS
1. Install [homebrew](https://docs.brew.sh/Installation)
2. Install GCC 13.3.1 with `brew install gcc@13`
3. Test GCC with `gcc --version` - this command should output information about the installed GCC compiler, including
its version number.

### Using CMake Presets

This repository is driven by CMake presets in `CMakePresets.json`. Build directories are created under `build/<preset>`.

Examples:
```
# List configure/build presets
cmake --list-presets=configure
cmake --list-presets=build

# Configure + build debug on Linux
cmake --preset linux-gcc-debug
cmake --build --preset linux-gcc-debug

# Configure + build release on macOS
cmake --preset macos-gcc-release
cmake --build --preset macos-gcc-release

# RISC-V cross build (Linux only with the cross toolchain installed)
cmake --preset riscv32-gcc-debug
cmake --build --preset riscv32-gcc-debug

# Clean rebuild using CMake's native reconfigure (CMake >= 3.24)
cmake --preset linux-gcc-debug --fresh
cmake --build --preset linux-gcc-debug

# Build a specific target
cmake --build --preset linux-gcc-debug --target gncAlgorithms

# Build with extra parallelism
cmake --build --preset linux-gcc-debug -- -j8
```

Note: `build_a.sh` and `build_all.sh` are legacy and not required when using CMake presets directly.

One can add a user-defined preset to CMakeUserPresets.json which is not added to the repository.
E.g.
```
{
  "version": 5,
  "cmakeMinimumRequired": {
    "major": 3,
    "minor": 24,
    "patch": 0
  },

  "configurePresets": [
    {
      "name": "with-eigen",
      "inherits": "macos-gcc-debug",
      "cacheVariables": {
        "EIGEN3_DIR": "/Users/joe/eigen"
      }
    }
  ],

  "buildPresets": [
    {
      "name": "with-eigen",
      "configurePreset": "with-eigen",
      "verbose": true
    }
  ]
}
```

## Static Analysis

This repository contains a `.clang-tidy` file which can be used within an IDE and is also used in the continuous
integration checks. It is also recommended to configure the MISRA checks in your IDE. Below is a list of excluded
MISRA rules.
- MISRA 5.10.1: A macro identifier shall additionally only be formed using characters in the ranges [A-Z], [0-9] and _
- MISRA 6.0.3: The only declarations in the global namespace should be main, namespace declarations and extern "C"
declarations
- MISRA 7-3-1: The global namespace shall only contain main, namespace declarations and extern "C" declarations
