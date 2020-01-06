cmake_minimum_required(VERSION 2.8.3)

project(pcl-realsense)

# set build type = Release mode
set(CMAKE_BUILD_TYPE Release)

message("\n" "=========================================")
message("Project: ${PROJECT_NAME} ")
message("=========================================")

# set the include directive in the same project folder
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# set corresponding libraries directories
set(PCL_DIR /opt/pcl-1.9.0/build)					# PCL build directory
set(RS_DIR /opt/librealsense-legacy)				# Realsense directory
set(RS_LIB_DIR ${RS_DIR}/build)						# Realsense build directory
set(RS_INCLUDE_DIR ${RS_DIR}/include)				# Realsense include directory

# Include dependencies of pcl 1.8.1 in project directory
set(CMAKE_MODULE_PATH ${PCL_DIR}/../cmake/Modules)

# set cmake for use std c++11 and output executable folder to bin
set(CMAKE_CXX_STANDARD 11)
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

##############################################################################
# PACKAGES
##############################################################################
message("***********************")
message("PCL PACKAGE")
message("***********************")

# find_package(PCL 1.8 PATHS ${PCL_DIR} REQUIRED COMPONENTS common io visualization)
find_package(PCL 1.9 PATHS ${PCL_DIR} QUIET)
if(PCL_FOUND)
    message(STATUS "PCL status:")
    message(STATUS "    version: ${PCL_VERSION}")
    message(STATUS "    include: ${PCL_DIR}")
else()
    message(WARNING "   PCL 1.9 not found, attempting 1.8...")
    find_package(PCL 1.8 REQUIRED QUIET)
    if(PCL_FOUND)
        message(STATUS "PCL status:")
        message(STATUS "    version: ${PCL_VERSION}")
        message(STATUS "    include: ${PCL_DIR}")
    else()
    	message(WARNING "   PCL 1.8 not found, attempting 1.7...")     
        find_package(PCL 1.7 REQUIRED QUIET)
    	if(PCL_FOUND)
        	message(STATUS "PCL status:")
        	message(STATUS "    version: ${PCL_VERSION}")
        	message(STATUS "    include: ${PCL_DIR}")
    	else()
        	message(FATAL_ERROR "    ERROR: PCL minimum required version 1.7. Not found")
    	endif()
    endif()
endif()

message("***********************")
message("Realsense: Legacy Library")
message("***********************")

find_library(REALSENSE_LIB realsense HINTS "${RS_LIB_DIR}" REQUIRED)
if(REALSENSE_LIB)    
    message(STATUS "realsense status:")
    message(STATUS "    dir: ${REALSENSE_LIB}") 
else()
    message(FATAL_ERROR "    ERROR: realsense not found")   
endif()

##############################################################################
# HEADERS
##############################################################################
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${RS_INCLUDE_DIR})
include(CheckFunctionExists)

# Use the compile definitions defined in PCL
add_definitions(${PCL_DEFINITIONS})

##############################################################################
# LIBRARIES PATH
##############################################################################
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${REALSENSE_LIB})

##############################################################################
# SOURCE CODE
##############################################################################
set(MAIN_SOURCE "main.cpp")
#set(MAIN_SOURCE "main_multple_ports.cpp")

##############################################################################
# EXECUTABLES
##############################################################################
add_executable(${PROJECT_NAME} ${MAIN_SOURCE})
add_executable(regis-test registration_test.cpp)
add_executable(stitching-test stitching_test.cpp)

##############################################################################
# TARGET LIBRARIES
##############################################################################
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${REALSENSE_LIB})
target_link_libraries(regis-test ${PCL_LIBRARIES})
target_link_libraries(stitching-test ${PCL_LIBRARIES} )
target_compile_options(${PROJECT_NAME} PRIVATE -w)

message("=========================================")
message("Project: ${PROJECT_NAME} COMPILED WITH CMAKE " ${CMAKE_VERSION})
message("=========================================")


