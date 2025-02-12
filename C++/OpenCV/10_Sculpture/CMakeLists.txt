########## INIT ####################################################################################

cmake_minimum_required( VERSION 3.5 FATAL_ERROR )

project( ArtisticStructureFromMotion )



########## SETTINGS ################################################################################

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/.." ) # Place the exec in the root directory


########## DEPENDENCIES ############################################################################

##### Packages #####

find_package( OpenCV REQUIRED )
if( OpenCV_FOUND )
    message( STATUS "OpenCV found: ${OpenCV_VERSION}" )
else()
    message( FATAL_ERROR "OpenCV not found" )
endif()

find_package( PCL 1.2 REQUIRED )

include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )



########## PROGRAMS ################################################################################

##### 00: Relative Pose #####
set( EXECUTABLE_NAME 00_two-image-kps.out )
add_executable( ${EXECUTABLE_NAME} 
                00_test_01.cpp 
                Structure.cpp Image.cpp)
target_link_libraries( ${EXECUTABLE_NAME} ${OpenCV_LIBS} )


##### 01: PCD from Two-Image Registration #####
# set( EXECUTABLE_NAME 01_pcd-show.out )
# add_executable( ${EXECUTABLE_NAME} 
#                 01_pcd-display.cpp 
#                 Structure.cpp Image.cpp)
# target_link_libraries( ${EXECUTABLE_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} )


##### 02: PCL Visualizer Demo #####
set( EXECUTABLE_NAME 02_visualizer_demo.out )
add_executable( ${EXECUTABLE_NAME} 02_pcd-test.cpp )
target_link_libraries( ${EXECUTABLE_NAME} ${PCL_LIBRARIES} )