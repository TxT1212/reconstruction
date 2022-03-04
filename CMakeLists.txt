cmake_minimum_required(VERSION 3.5)
# set (CMAKE_CXX_STANDARD 11)
set(CMAKE_C_COMPILER /usr/bin/gcc-5)
set(CMAKE_CXX_COMPILER /usr/bin/g++-5)
project(BundleFusionDenseRecon)
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
    list(APPEND Open3D_LIBRARIES dl)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# set(CUDA_NVCC_FLAGS " ${CUDA_NVCC_FLAGS};  -gencode arch=compute_61,code=sm_61; -G; -g")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS} -fopenmp")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS} ")

add_definitions(-DUNIX)
add_compile_options(-Wno-deprecated-declarations)
add_compile_options(-Wno-unused-result)
add_definitions(-O3)
# cuda
find_package(CUDA REQUIRED MODULE)
if(CUDA_FOUND)
    # message(${CUDA_INCLUDE_DIRS})
    # message(${CUDA_LIBRARIES})
    # message(${CUDA_LIBRARY_PATH})
    include_directories(${CUDA_INCLUDE_DIRS})
endif()

set(CUDA_SOURCE_PROPERTY_FORMAT OBJ)
set(CUDA_SEPARABLE_COMPILATION ON)
include_directories(${bCUDA_INCLUDE_DIRS})
set(CUDA_PROPAGATE_HOST_FLAGS OFF)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=sm_61;-O3;-G;-g;-std=c++11)

# Eigen
include_directories("/usr/include/eigen3")

include_directories("/usr/local/cuda-10.1/targets/x86_64-linux/include/")
#glog
find_package(Glog REQUIRED)

#opencv
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Open3D REQUIRED)

include_directories(${Open3D_INCLUDE_DIRS})
link_directories(${Open3D_LIBRARY_DIRS})

cuda_add_executable(BundleFusionDenseRecon 
    main.cpp
    CUDAFilter.cu
)
cuda_add_executable(reconstruct_tof 
    reconstruct_tof.cc
    CUDAFilter.cu
)
cuda_add_executable(reconstruct_structure
    reconstruct_structure.cc
    CUDAFilter.cu
)
add_executable(depthdiff depth_difference.cc)
target_link_libraries(depthdiff ${OpenCV_LIBS})
# message(${Open3D_LIBRARIES})
target_link_libraries(BundleFusionDenseRecon
    ${CUDA_LIBARIES}
    ${Glog_LIBRARIES}
    ${OpenCV_LIBS}
    ${Open3D_LIBRARIES}
)
target_link_libraries(reconstruct_tof
    ${CUDA_LIBARIES}
    ${Glog_LIBRARIES}
    ${OpenCV_LIBS}
    ${Open3D_LIBRARIES}
)
target_link_libraries(reconstruct_structure
    ${CUDA_LIBARIES}
    ${Glog_LIBRARIES}
    ${OpenCV_LIBS}
    ${Open3D_LIBRARIES}
)
option(BUILD_SHARED_LIBS "Whether Open3D was build as shared library" OFF)

# add_executable(try try.cc)

# target_link_libraries(try
# ${Glog_LIBRARIES}
# ${OpenCV_LIBS}
# )