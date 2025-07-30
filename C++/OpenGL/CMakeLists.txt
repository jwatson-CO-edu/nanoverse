########## INIT ####################################################################################

cmake_minimum_required( VERSION 3.5 FATAL_ERROR )

project( NanoverseCppOpenGL )


########## SETTINGS ################################################################################

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -no-pie -O3 -Wall -lGL -lGLU -lglut -lm" )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/.." ) # Place the exec in the root directory
include( CMakePrintHelpers )



########## DEPENDENCIES ############################################################################

##### Packages #####
find_package( PkgConfig REQUIRED )
find_package( OpenGL REQUIRED )
find_package( GLUT REQUIRED )

find_package( OpenCV REQUIRED )
if( OpenCV_FOUND )
    message( STATUS "OpenCV found: ${OpenCV_VERSION}" )
else()
    message( FATAL_ERROR "OpenCV not found" )
endif()

cmake_print_variables( PROJECT_SOURCE_DIR )

##### Local Libraries #####
set( CFITSIO_INCLUDE_DIRS /opt/fits/include )
set( CFITSIO_LIBRARIES    /opt/fits/lib     )

add_library( OGL_LIB
             SHARED
             OGL_utils.cpp
             LightSource.cpp
             load_assets.cpp )

target_include_directories( OGL_LIB PUBLIC include ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} )
target_link_libraries( OGL_LIB PUBLIC ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} )

add_library( PUNCH_LIB
             SHARED
             astrophysics/PUNCH/PUNCH.cpp )

target_compile_options( PUNCH_LIB PRIVATE "-L${CFITSIO_LIBRARIES}" -lcfitsio -lm )
target_include_directories( PUNCH_LIB PUBLIC include ${CFITSIO_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} )
target_link_libraries( PUNCH_LIB PUBLIC ${CFITSIO_LIBRARIES} ${OpenCV_LIBS} )



########## PROGRAMS ################################################################################

##### OpenGL Toys #########################################################

#### 00: Compilation Test #####
set( EXECUTABLE_NAME 00_restart.out )
add_executable( ${EXECUTABLE_NAME} 00_restart.cpp  )
target_include_directories( ${EXECUTABLE_NAME} PRIVATE include ${OPENGL_INCLUDE_DIRS} )
target_link_libraries( ${EXECUTABLE_NAME} PRIVATE OGL_LIB ${OPENGL_LIBRARIES} )

##### PUNCH Solar Observatory #############################################

#### 02: Data Visualization Test #####
set( EXECUTABLE_NAME 02_viz-one-pair.out )
add_executable( ${EXECUTABLE_NAME} astrophysics/PUNCH/02_viz-one-pair.cpp )
target_include_directories( ${EXECUTABLE_NAME} PRIVATE include ${CFITSIO_INCLUDE_DIRS} )
target_link_libraries( ${EXECUTABLE_NAME} PRIVATE OGL_LIB PUNCH_LIB )
target_compile_options( ${EXECUTABLE_NAME} PRIVATE "-L${CFITSIO_LIBRARIES}" -lcfitsio -lm )



