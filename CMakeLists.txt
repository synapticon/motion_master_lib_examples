# Top-level CMake project configuration for MotionMasterLibExamples

# Specify the minimum required CMake version to ensure compatibility
cmake_minimum_required(VERSION 3.28)

# Set CMP0167 to OLD to retain support for the legacy FindBoost module.
# This suppresses warnings introduced in CMake 3.29+ where FindBoost was removed.
# Required for compatibility with vcpkg's current Boost wrapper which still uses _find_package(Boost).
cmake_policy(SET CMP0167 OLD)

# Define the Windows API target version as Windows 10/11 (0x0A00)
# Enables use of Windows 10+ specific features and APIs
if(WIN32)
  add_definitions(-D_WIN32_WINNT=0x0A00)
endif()

# Declare the project and specify that it uses C++
project(MotionMasterLibExamples LANGUAGES C CXX)

# Set C++ standard to C++20 and make it required
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add loguru logging library as a subdirectory
add_subdirectory(loguru)

# Find required packages
find_package(Boost REQUIRED COMPONENTS asio)
find_package(kubazip CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

# Create executable target from source files
add_executable(spoe src/spoe.cpp)

# Add the include directory for header files
target_include_directories(spoe PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Link Motion Master static libraries based on platform and build configuration
if(WIN32)
  target_link_libraries(spoe PRIVATE
    $<$<CONFIG:Debug>:${CMAKE_SOURCE_DIR}/lib/comm_debug.lib>
    $<$<CONFIG:Release>:${CMAKE_SOURCE_DIR}/lib/comm.lib>
    $<$<CONFIG:Debug>:${CMAKE_SOURCE_DIR}/lib/core_debug.lib>
    $<$<CONFIG:Release>:${CMAKE_SOURCE_DIR}/lib/core.lib>
  )
else()
  target_link_libraries(spoe PRIVATE
    $<$<CONFIG:Debug>:${CMAKE_SOURCE_DIR}/lib/libcomm_debug.a>
    $<$<CONFIG:Release>:${CMAKE_SOURCE_DIR}/lib/libcomm.a>
    $<$<CONFIG:Debug>:${CMAKE_SOURCE_DIR}/lib/libcore_debug.a>
    $<$<CONFIG:Release>:${CMAKE_SOURCE_DIR}/lib/libcore.a>
  )
endif()

# Link common third-party libraries
target_link_libraries(spoe PRIVATE
  Boost::asio
  kubazip::kubazip
  loguru
  nlohmann_json::nlohmann_json
)

# On Windows, link against multimedia and sockets system libraries
if(WIN32)
  target_link_libraries(spoe PRIVATE winmm ws2_32)
endif()
