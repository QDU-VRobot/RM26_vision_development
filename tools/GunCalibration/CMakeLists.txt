cmake_minimum_required(VERSION 3.5) # Minimum CMake version required
# Set the project name and version
project(cv) # Project name is 'cv'


set(CMAKE_CXX_STANDARD 11)

set(CMAKE_EXPORT_COMPILE_COMMANDS True)

# Find all package that are required
find_package( OpenCV REQUIRED ) # Find OpenCV package
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(MVS REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBUSB_1.0 REQUIRED libusb-1.0)
# find_package(LibUSB 1.0 REQUIRED)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_executable(GunCalibrate 
    src/HikCamera.cpp
    src/logger.cpp
    src/GunCalibrate.cpp
)

target_include_directories(GunCalibrate PUBLIC ${MVS_INCLUDE_DIRS})
target_link_libraries(GunCalibrate ${OpenCV_LIBS} fmt::fmt spdlog::spdlog ${LIBUSB_1.0_LIBRARIES} ${MVS_LIBS} )
# add the executable target









