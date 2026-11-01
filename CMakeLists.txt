cmake_minimum_required(VERSION 3.16)
project(UnifiedEngineeringWorkbench VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Optimization flags
if(MSVC)
    add_compile_options(/W4 /O2 /permissive-)
else()
    add_compile_options(-Wall -Wextra -O2)
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/backend/include
)

# Source files
set(BACKEND_SOURCES
    backend/src/common/csv_utils.cpp
    backend/src/common/linear_algebra.cpp
    backend/src/common/validation.cpp
    backend/src/geometry/stl_parser.cpp
    backend/src/geometry/csv_temperature_parser.cpp
    backend/src/thermal/thermal_solver.cpp
    backend/src/performance/batch_generator.cpp
    backend/src/performance/benchmark.cpp
    backend/src/hardware/microcontroller_adapter.cpp
    backend/src/hardware/fpga_adapter.cpp
    backend/src/hardware/jtag_adapter.cpp
    backend/src/backpropagation/neural_network.cpp
    backend/src/api/server.cpp
    backend/src/main.cpp
)

# Executable target
add_executable(workbench_backend ${BACKEND_SOURCES})

# Windows socket and thread libraries
if(WIN32)
    target_link_libraries(workbench_backend PRIVATE ws2_32)
    if(MINGW)
        target_link_libraries(workbench_backend PRIVATE pthread)
    endif()
endif()

# Standalone Launcher target
add_executable(Launch scripts/Launch.cpp)
if(WIN32)
    target_link_libraries(Launch PRIVATE shell32)
endif()

