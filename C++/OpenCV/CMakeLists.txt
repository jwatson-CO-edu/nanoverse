########## INIT ####################################################################################

cmake_minimum_required( VERSION 3.10 ) # Set minimum CMake version

project( CppOpenCVExperiments ) # Set project name



########## SETTINGS ################################################################################

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/.." ) # Place the exec in the root directory



########## DEPENDENCIES ############################################################################

##### System #####

find_package( OpenCV REQUIRED )
if(OpenCV_FOUND)
    message( STATUS "OpenCV found: ${OpenCV_VERSION}" )
else()
    message( FATAL_ERROR "OpenCV not found" )
endif()

find_package( matioCpp REQUIRED )


##### Local #####

# add_subdirectory( matio )



########## PROGRAMS ################################################################################
set( EXECUTABLE_NAME 00_asteroid-SfM.out )
add_executable( ${EXECUTABLE_NAME} 
                00_Asteroid.cpp
                SfM.cpp image_proc.cpp helpers.cpp )
target_link_libraries( ${EXECUTABLE_NAME} PUBLIC ${OpenCV_LIBS} PRIVATE matioCpp::matioCpp )
# target_link_libraries( build-asteroid.out PRIVATE matioCpp::matioCpp )
