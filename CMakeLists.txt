cmake_minimum_required(VERSION 2.8.8)
project(e57_pcl)

set (e57_pcl_VERSION_MAJOR 0)
set (e57_pcl_VERSION_MINOR 1)

include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/include")
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

if(CMAKE_COMPILER_IS_GNUCXX)
	add_definitions(-fPIC)
	add_definitions(-O3)
	add_definitions(-g)
	add_definitions(-std=c++1y)
    add_definitions(-Wall)
    add_definitions(-Wno-unused-function)
    add_definitions(-Wno-unused-local-typedefs)
    add_definitions(-Wno-deprecated-declarations)
endif()

find_package(Eigen)
find_package(PCL COMPONENTS common io search filters keypoints features)
find_package(Boost COMPONENTS system program_options filesystem random)
find_package(LibE57)

file (GLOB_RECURSE obj RELATIVE "${PROJECT_SOURCE_DIR}" "src/*.cpp")
if (EIGEN_FOUND AND PCL_FOUND AND Boost_FOUND AND LIBE57_FOUND)
	include_directories(${EIGEN_INCLUDE_DIRS})
	include_directories(${PCL_INCLUDE_DIRS})
	include_directories(${Boost_INCLUDE_DIRS})
	include_directories(${LIBE57_INCLUDE_DIRS})

    add_library(e57_pcl SHARED ${obj})
    target_link_libraries(e57_pcl ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${LIBE57_LIBRARIES} "dl")

	# install binary
    install (TARGETS e57_pcl DESTINATION lib)
	# install header
    install (DIRECTORY include/ DESTINATION include/e57_pcl)
    # install find module
    file (GLOB find_modules RELATIVE "${PROJECT_SOURCE_DIR}" "cmake/modules/self/*.cmake")
    install (FILES ${find_modules} DESTINATION share/E57PCL)
endif()
