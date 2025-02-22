########## INIT ####################################################################################

cmake_minimum_required( VERSION 3.5 FATAL_ERROR )

project( ArtisticStructureFromMotion )



########## SETTINGS ################################################################################

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/.." ) # Place the exec in the root directory
include( CMakePrintHelpers )



########## DEPENDENCIES ############################################################################

##### Packages #####

find_package( OpenCV REQUIRED )
if( OpenCV_FOUND )
    message( STATUS "OpenCV found: ${OpenCV_VERSION}" )
else()
    message( FATAL_ERROR "OpenCV not found" )
endif()

find_package( VTK REQUIRED )
find_package( PCL 1.2 REQUIRED )

cmake_print_variables( PROJECT_SOURCE_DIR )

##### Local Libraries #####

add_library( SFM_LIB
             Fit.cpp 
             Visualize.cpp 
             Structure.cpp 
             Image.cpp )

target_include_directories( SFM_LIB PUBLIC ${VTK_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )

target_link_libraries( SFM_LIB 
                       ${PCL_LIBRARIES} 
                       ${VTK_LIBRARIES} 
                       ${OpenCV_LIBS} )

########## PROGRAMS ################################################################################

##### 00: Relative Pose #####
set( EXECUTABLE_NAME 00_two-image-kps.out )
add_executable( ${EXECUTABLE_NAME} 00_test_01.cpp )
# target_include_directories( ${EXECUTABLE_NAME} PUBLIC ${VTK_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
target_link_libraries( ${EXECUTABLE_NAME} SFM_LIB )


##### 01: PCD from Two-Image Registration #####
set( EXECUTABLE_NAME 01_pcd-show.out )
add_executable( ${EXECUTABLE_NAME} 01_pcd-display.cpp )
# target_include_directories( ${EXECUTABLE_NAME} PUBLIC ${VTK_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
# target_link_libraries( ${EXECUTABLE_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} )
target_link_libraries( ${EXECUTABLE_NAME} SFM_LIB )


##### 02: PCL Visualizer Demo #####
set( EXECUTABLE_NAME 02_visualizer_demo.out )
add_executable( ${EXECUTABLE_NAME} 02_pcd-test.cpp )
# target_include_directories( ${EXECUTABLE_NAME} PUBLIC ${VTK_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
# target_link_libraries( ${EXECUTABLE_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} )
target_link_libraries( ${EXECUTABLE_NAME} SFM_LIB )


##### 03: PCD from Six-Image Registration #####
set( EXECUTABLE_NAME 03_six-image-pcd.out )
add_executable( ${EXECUTABLE_NAME} 03_six-image.cpp )
# target_include_directories( ${EXECUTABLE_NAME} PUBLIC ${VTK_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
# target_link_libraries( ${EXECUTABLE_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} )
target_link_libraries( ${EXECUTABLE_NAME} SFM_LIB )


##### 04: Clean Clouds from Six-Image Registration #####
set( EXECUTABLE_NAME 04_six-img-cleanup.out )
add_executable( ${EXECUTABLE_NAME} 04_six-cleanup.cpp )
# target_include_directories( ${EXECUTABLE_NAME} PUBLIC ${VTK_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
# target_link_libraries( ${EXECUTABLE_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} )
target_link_libraries( ${EXECUTABLE_NAME} SFM_LIB )
