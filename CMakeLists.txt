cmake_minimum_required(VERSION 3.17)
project(ray_casting)

set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_VERBOSE_MAKEFILE ON)

# OpenCV
set(OpenCV_DIR ~/Documents/opencv)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# PNG
find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIRS})
include_directories(${LIBVISO2_SRC_DIR})
link_directories(${PNG_LIBRARY_DIRS})
add_definitions(${PNG_DEFINITIONS})

# Eigen
include_directories($ENV{EIGEN})

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
        ${OpenCV_LIBS}
)

# CUDA
if (DEFINED ENV{CUDA_HOME})
    message("CUDA found at $ENV{CUDA_HOME}")
    set(CMAKE_CUDA_COMPILER $ENV{CUDA_HOME}/bin/nvcc)
    enable_language(CUDA)
    include_directories($ENV{CUDA_HOME}/include ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

    # include dir
    include_directories(include)

    file(GLOB utils_cpp utils/*.cpp)
    file(GLOB utils_cu utils/*.cu)
    file(GLOB tsdf_cpp TSDF/*cpp)
    file(GLOB tsdf_cu TSDF/*cu)

    add_executable(
            ray_casting_gpu
            main_gpu.cpp
            ${utils_cu}
            ${tsdf_cu}
            RayCaster/GPURaycaster.cu
            RayCaster/Raycaster.cpp
    )
    target_link_libraries(
            ray_casting_gpu
            utils
            ${PNG_LIBRARY}
            ${OpenCV_LIBS}
    )

    # necessary as the kernel exceeds maximum 64k
    target_compile_options(
            ray_casting_gpu
            PRIVATE
            $<$<COMPILE_LANGUAGE:CUDA>:--maxrregcount=64 -use_fast_math>
    )

    set_target_properties(
            ray_casting_gpu
            PROPERTIES
            CUDA_SEPARABLE_COMPILATION ON
    )

else ()
    message("CUDA not found. Build for CPU only.")
endif ()
