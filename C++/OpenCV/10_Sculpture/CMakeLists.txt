########## INIT ####################################################################################

cmake_minimum_required( VERSION 3.5 FATAL_ERROR )

project( ArtisticStructureFromMotion )



########## SETTINGS ################################################################################

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/.." ) # Place the exec in the root directory


########## DEPENDENCIES ############################################################################

##### Packages #####

find_package( PCL 1.2 REQUIRED )

include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )



########## PROGRAMS ################################################################################

set( EXECUTABLE_NAME pcl_visualizer_demo.out )
add_executable( ${EXECUTABLE_NAME} 02_pcd-test.cpp )
target_link_libraries( ${EXECUTABLE_NAME} ${PCL_LIBRARIES} )