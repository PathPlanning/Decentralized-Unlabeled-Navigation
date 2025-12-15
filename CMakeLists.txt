cmake_minimum_required(VERSION 3.27)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0148 OLD)

if(POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
endif()

project(ctp-swap)

add_compile_definitions(M_PIl=0xc.90fdaa22168c235p-2L)
#define M_PIl       0xc.90fdaa22168c235p-2L


set(BUILD_SHARED_LIBS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -w -O0  -fno-fast-math  -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w -O0 -fPIC")


# add_subdirectory(external/tinyxml2)
# add_subdirectory(external/kdtree)
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
add_subdirectory(libs/trivis)

# find_package(pybind11 CONFIG REQUIRED)

# set_target_properties(construction_time PROPERTIES EXCLUDE_FROM_ALL TRUE)
# set_target_properties(error_test PROPERTIES EXCLUDE_FROM_ALL TRUE)
# set_target_properties(toy_test PROPERTIES EXCLUDE_FROM_ALL TRUE)
set(CTP_SWAP_LIB_HEADERS
        ${PROJECT_SOURCE_DIR}/include/action.hpp
        ${PROJECT_SOURCE_DIR}/include/message.hpp
        ${PROJECT_SOURCE_DIR}/include/agent.hpp
        ${PROJECT_SOURCE_DIR}/include/simple_agent.hpp
        ${PROJECT_SOURCE_DIR}/include/ctpswap_agent.hpp
        ${PROJECT_SOURCE_DIR}/include/direct_planner.hpp
        ${PROJECT_SOURCE_DIR}/include/follower.hpp
        ${PROJECT_SOURCE_DIR}/include/direct_follower.hpp
        ${PROJECT_SOURCE_DIR}/include/orca_follower.hpp
        ${PROJECT_SOURCE_DIR}/include/theta_planner.hpp
        ${PROJECT_SOURCE_DIR}/include/grid_map.hpp
        ${PROJECT_SOURCE_DIR}/include/theta_star.hpp
        ${PROJECT_SOURCE_DIR}/include/node.hpp
        ${PROJECT_SOURCE_DIR}/include/search_tree.hpp
        ${PROJECT_SOURCE_DIR}/include/visibility_planner.hpp
        ${PROJECT_SOURCE_DIR}/include/environment_description.hpp
        ${PROJECT_SOURCE_DIR}/include/visibility_graph.hpp
        ${PROJECT_SOURCE_DIR}/include/dijkstra.hpp


)

set(CTP_SWAP_LIB_SOURCES
        ${PROJECT_SOURCE_DIR}/src/simple_agent.cpp
        ${PROJECT_SOURCE_DIR}/src/ctpswap_agent.cpp
        ${PROJECT_SOURCE_DIR}/src/direct_planner.cpp
        ${PROJECT_SOURCE_DIR}/src/direct_follower.cpp
        ${PROJECT_SOURCE_DIR}/src/orca_follower.cpp
        ${PROJECT_SOURCE_DIR}/src/theta_planner.cpp
        ${PROJECT_SOURCE_DIR}/src/grid_map.cpp
        ${PROJECT_SOURCE_DIR}/src/theta_star.cpp
        ${PROJECT_SOURCE_DIR}/src/visibility_planner.cpp
        ${PROJECT_SOURCE_DIR}/src/visibility_graph.cpp
        ${PROJECT_SOURCE_DIR}/src/dijkstra.cpp

)

add_library(ctp_swap_lib ${CTP_SWAP_LIB_HEADERS} ${CTP_SWAP_LIB_SOURCES})

target_include_directories(ctp_swap_lib
        PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)


target_compile_definitions(ctp_swap_lib PUBLIC -DUSE_CBLAS)
target_link_libraries(ctp_swap_lib PUBLIC ${LAPACK_LIBRARIES})

target_link_libraries(ctp_swap_lib PUBLIC xtensor)
target_link_libraries(ctp_swap_lib PUBLIC xtensor-blas)
target_link_libraries(ctp_swap_lib PUBLIC xsimd)
target_link_libraries(ctp_swap_lib PUBLIC RVO)
target_link_libraries(ctp_swap_lib PUBLIC Trivis)


pybind11_add_module(_ctp_swap python_bind/converters.hpp python_bind/converters.cpp python_bind/ctp_swap.cpp NO_EXTRAS)
target_link_libraries(_ctp_swap PUBLIC xtensor-python Python::NumPy)
target_link_libraries(_ctp_swap PUBLIC ctp_swap_lib Trivis)


# target_link_libraries(dec_unav_lib tinyxml2 KDTree RVO)

# add_executable(single ${PROJECT_SOURCE_DIR}/bin/single.cpp)
# add_executable(series ${PROJECT_SOURCE_DIR}/bin/series.cpp)

# target_link_libraries(single dec_unav_lib tinyxml2 KDTree RVO)
# target_link_libraries(series dec_unav_lib tinyxml2 KDTree RVO)
