cmake_minimum_required(VERSION 3.0.2)
project(librobo2d)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -ggdb")

find_package(Ceres REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(OpenCV 3.2 REQUIRED)
find_package(glog  REQUIRED)
find_package(libnabo REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(ROBOLIB_INCLUDE_DIR ${catkin_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE PATH 
    "project include location" FORCE)
# list(APPEND MSAL3D_INCLUDE_DIR ${catkin_INCLUDE_DIRS})
message("ROBOLIB_INCLUDE_DIR: " ${ROBOLIB_INCLUDE_DIR})

file(GLOB_RECURSE common CONFIGURE_DEPENDS src/Filter/*.cpp src/Map/*.cpp) 
file(GLOB_RECURSE scan_matcher_src CONFIGURE_DEPENDS src/ScanMatcher/*.cpp) 

add_library(robo2d_lib SHARED ${common} ${scan_matcher_src})

target_include_directories(robo2d_lib PUBLIC 
    include
    include/robo2d/
    ${EIGEN3_INCLUDE_DIR}
    ${GLOG_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${libnabo_INCLUDE_DIRS}
)

# message("4567893456789234567892345678923456789234567891234567, " ${OpenCV_INCLUDE_DIRS})
# target_link_directories(robo2d_lib PRIVATE
#     /home/lwh/cross_compile/usr/lib/aarch64-linux-gnu
#     /home/lwh/cross_compile/usr/lib
# )

target_link_libraries(robo2d_lib PUBLIC 
    ${PCL_LIBRARYS}
    ${OpenCV_LIBS}
    glog::glog
    libnabo::nabo
)

# message("4567893456789234567892345678923456789234567891234567, " ${OpenCV_LIBS})

add_executable(test_csm_grid src/test/test_csm_grid.cpp)
target_link_libraries(test_csm_grid
    robo2d_lib
)

#############
## INSTALL ##
#############
install(TARGETS robo2d_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}   # .a
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}   # .so
)

# install(DIRECTORY include/msa2d
# DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})


