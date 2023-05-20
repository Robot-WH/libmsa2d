cmake_minimum_required(VERSION 3.0.2)
project(libmsa2d)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Ceres REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(OpenCV 3.0 QUIET)
find_package(glog  REQUIRED)
find_package(libnabo REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(MSALib_INCLUDE_DIR ${catkin_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE PATH 
    "project include location" FORCE)
# list(APPEND MSAL3D_INCLUDE_DIR ${catkin_INCLUDE_DIRS})
message("MSALib_INCLUDE_DIR: " ${MSALib_INCLUDE_DIR})

file(GLOB_RECURSE map_src CONFIGURE_DEPENDS src/Map/*.cpp) 
add_library(map_lib SHARED ${map_src})
target_include_directories(map_lib PUBLIC 
    include
    ${EIGEN3_INCLUDE_DIR}
    ${GLOG_INCLUDE_DIRS}
    ${libnabo_INCLUDE_DIRS}
)
target_link_libraries(map_lib PUBLIC 
    glog::glog
    libnabo::nabo
)

# GLOB_RECURSE 会递归的找到子目录文件
# 加了CONFIGURE_DEPENDS，如果有新文件添加，cmake --build 会自动运行 cmake -B build 进行更新
file(GLOB_RECURSE scan_matcher_src CONFIGURE_DEPENDS src/ScanMatcher/*.cpp) 
add_library(scan_matcher_lib SHARED ${scan_matcher_src})
target_include_directories(scan_matcher_lib PUBLIC 
    include
    ${EIGEN3_INCLUDE_DIR}
    ${GLOG_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(scan_matcher_lib PUBLIC 
    ${PCL_LIBRARYS}
    glog::glog
)

#############
## INSTALL ##
#############

install(TARGETS map_lib scan_matcher_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}   # .a
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}   # .so
)

install(DIRECTORY include/msa2d
DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})


