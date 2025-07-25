########## INIT ####################################################################################

cmake_minimum_required( VERSION 3.5 FATAL_ERROR )

project( NanoverseCppOpenGL )


########## SETTINGS ################################################################################

set( CMAKE_CXX_STANDARD 17 )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -lglut -lGLU -lGL -lm")
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/.." ) # Place the exec in the root directory
include( CMakePrintHelpers )



########## DEPENDENCIES ############################################################################

##### Packages #####
find_package( PkgConfig REQUIRED )
find_package( OpenGL REQUIRED )

# Find OpenGL utilities 
find_library(GLU_LIBRARY GLU REQUIRED)
# Find GLUT for window management 
find_library(GLUT_LIBRARY glut REQUIRED)

find_package( OpenCV REQUIRED )
if( OpenCV_FOUND )
    message( STATUS "OpenCV found: ${OpenCV_VERSION}" )
else()
    message( FATAL_ERROR "OpenCV not found" )
endif()

cmake_print_variables( PROJECT_SOURCE_DIR )

##### Local Libraries #####

add_library( OGL_LIB
             geometry.cpp
             LightSource.cpp
            #  load_assets.cpp 
             OGL_utils.cpp )

target_include_directories( OGL_LIB PUBLIC ${OPENGL_INCLUDE_DIRS} )

########## PROGRAMS ################################################################################

##### OpenGL Toys #########################################################

##### 00: Relative Pose #####
# set( EXECUTABLE_NAME 00_restart.out )
# add_executable( ${EXECUTABLE_NAME} 00_restart.cpp )
# target_link_libraries( ${EXECUTABLE_NAME} OGL_LIB )

##### PUNCH Solar Observatory #########################################################
