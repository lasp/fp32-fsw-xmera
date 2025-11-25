# Repository Name
This repository contains the GNC FSW algorithms.

## Getting Started
This repository has two primary usages and therefore two build paths:
1. To compile the FSW algorithms as modules for the Ximera simulation tool.
2. To compile the FSW algorithms for the GNC target platform.

The build Ximera build path is controlled by the Ximera build system. It expects the directory structure:
- algorithms/
  - msgPayloadDef/
  - alg1/
    - module.i
    - module.h
    - module.cpp
  - alg2/
    - etc.

The flight software build path is controlled by the CMake build system and associated cmake files in this repository.
This build path is described below.

### Dependencies:
- Eigen for freestanding C++ (it's assumed that the repo is at ../ relative to this repo)

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

### Using Build.sh
```
build.sh — drives builds via CMakePresets.json

Examples:
  ./build.sh --list
  ./build.sh linux-gcc-debug/release
  ./build.sh macos-gcc-debug/release
  ./build.sh riscv32-gcc-debug/release (this option only works on Linux with installed cross compiler)
  ./build.sh --configure linux-gcc-debug --build linux-gcc-debug --fresh
  ./build.sh macos-gcc-debug --archive lib/libgncAlgorithms.a

Notes:
- Uses your CMakePresets.json configure/build presets.
- --fresh uses CMake's native reconfigure (CMake >= 3.24).
- The script chooses a sensible default preset if none is provided.
```

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
MISRA 7-3-1: The global namespace shall only contain main, namespace declarations and extern "C" declarations
MISRA 6.0.3: The only declarations in the global namespace should be main, namespace declarations and extern "C"
declarations
