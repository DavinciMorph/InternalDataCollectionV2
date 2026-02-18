cmake_minimum_required(VERSION 3.16)
project(ads1299_acquisition LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags for Cortex-A72 (Pi 4 / CM4)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=cortex-a72 -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

# Dead code elimination
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")

# LTO for cross-file inlining (ring buffer push/pop into engine and CSV writer)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)

# Default to Release build
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Sources
set(SOURCES
    src/main.cpp
    src/hardware/spi_bus.cpp
    src/hardware/i2c_device.cpp
    src/hardware/tca9534.cpp
    src/ads1299/spi_device.cpp
    src/ads1299/controller.cpp
    src/acquisition/engine.cpp
    src/acquisition/bus_worker.cpp
    src/acquisition/drdy_poller.cpp
    src/logging/csv_writer.cpp
    src/streaming/protocol.cpp
    src/streaming/server.cpp
)

add_executable(ads1299_acquire ${SOURCES})

target_include_directories(ads1299_acquire PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Link pthreads
find_package(Threads REQUIRED)

# Find LZ4 (liblz4-dev must be installed: apt install liblz4-dev)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LZ4 REQUIRED liblz4)

target_include_directories(ads1299_acquire PRIVATE ${LZ4_INCLUDE_DIRS})
target_link_libraries(ads1299_acquire PRIVATE Threads::Threads ${LZ4_LIBRARIES})
