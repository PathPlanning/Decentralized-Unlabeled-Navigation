cmake_minimum_required(VERSION 3.17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

project(experiment-utils)

set(BUILD_SHARED_LIBS OFF) 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -w -O0  -fno-fast-math  -fPIC")


add_subdirectory(libs/rvo2)

add_compile_options(-lcblas)
add_compile_options(-lopenblas)

find_package(Python REQUIRED COMPONENTS Interpreter Development NumPy)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)





message(STATUS "BLAS VENDOR:    " ${BLA_VENDOR})
message(STATUS "BLAS LIBRARIES: " ${BLAS_LIBRARIES})


set(XTENSOR_USE_XSIMD ON)

add_subdirectory(libs/pybind11)
add_subdirectory(libs/xsimd)
add_subdirectory(libs/xtl)
add_subdirectory(libs/xtensor)
add_subdirectory(libs/xtensor-blas)
add_subdirectory(libs/xtensor-python)


set(CTP_SWAP_LIB_HEADERS
    ${PROJECT_SOURCE_DIR}/include/experiment.hpp
)

set(CTP_SWAP_LIB_SOURCES
    ${PROJECT_SOURCE_DIR}/src/experiment.cpp
)

add_library(experiment_utils_lib ${CTP_SWAP_LIB_HEADERS} ${CTP_SWAP_LIB_SOURCES})

target_include_directories(experiment_utils_lib
    PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)


target_compile_definitions(experiment_utils_lib PUBLIC -DUSE_CBLAS)
target_link_libraries(experiment_utils_lib ${LAPACK_LIBRARIES})

target_link_libraries(experiment_utils_lib xtensor)
target_link_libraries(experiment_utils_lib xtensor-blas)
target_link_libraries(experiment_utils_lib xsimd)
target_link_libraries(experiment_utils_lib RVO)


pybind11_add_module(experiment_utils python_bind/cpp_utils.cpp NO_EXTRAS)
target_link_libraries(experiment_utils PRIVATE xtensor-python Python::NumPy)
target_link_libraries(experiment_utils PRIVATE experiment_utils_lib)