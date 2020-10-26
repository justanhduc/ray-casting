cmake_minimum_required(VERSION 3.17)
project(ray_casting)

set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_VERBOSE_MAKEFILE ON)

# PNG
find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIRS})
include_directories(${LIBVISO2_SRC_DIR})
link_directories(${PNG_LIBRARY_DIRS})
add_definitions(${PNG_DEFINITIONS})

# Eigen
include_directories(/home/justanhduc/Documents/eigen)

# include dir
include_directories(include)

file(GLOB utils_cpp utils/*.cpp)
file(GLOB tsdf_cpp TSDF/*.cpp)
add_library(
        utils
        ${utils_cpp}
        ${tsdf_cpp}
)

add_library(
        rc
        RayCaster/Raycaster.cpp
)
target_link_libraries(
        rc
        utils
        ${PNG_LIBRARY}
)

add_executable(
        raycasting
        main.cpp
        TSDF/TSDFUtilitiesCPU.cpp include/TSDFUtilitiesCPU.h)
target_link_libraries(
        raycasting
        utils
        rc
)
