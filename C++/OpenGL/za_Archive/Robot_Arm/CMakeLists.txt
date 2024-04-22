# TOPLEVEL CMAKELISTS

# ~ Project Setup ~
cmake_minimum_required( VERSION 2.6 )
project( HW3 )

message( "" )
message( "##### Project: ${PROJECT_NAME} #####" )
message( "" )

# === COMPILER SETTINGS ====================================================================================================================

# ~~ Options and Flags ~~

# ~ Flags we can all agree on ~
set( COMMON_FLAGS "-O3 -Wall -lm" )

# Platform-specific flags
if(WIN32)
    message( "Setting compiler flags for Windows ..." )
	set( MORE_FLAGS "${COMMON_FLAGS} -lglut32cu -lglu32 -lopengl32" )
endif(WIN32)

if(APPLE)
	message( "Setting compiler flags for Apple ..." )
	set( MORE_FLAGS "${COMMON_FLAGS} -Wno-deprecated-declarations -framework GLUT -framework OpenGL" )
endif(APPLE)

if(UNIX)
    message( "Setting compiler flags for Unix/Linux ..." )
    set( MORE_FLAGS "${COMMON_FLAGS} -lglut -lGLU -lGL" )
endif(UNIX)


# ~~ Configure Compiler ~~
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ${MORE_FLAGS}" ) # This will prevent g++ from getting upset about default values
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MORE_FLAGS}" ) # This will prevent g++ from getting upset about default values

# ___ END COMPILER _________________________________________________________________________________________________________________________


# === PACKAGE INCLUDES =====================================================================================================================

# ~~ Locate Packages ~~

find_package( OpenGL REQUIRED ) # Make sure OpenGL is available for direct calls
if( NOT OPENGL_FOUND )
    message( "ERROR: OpenGL not found" )
endif( NOT OPENGL_FOUND )

find_package( GLUT REQUIRED )
if( NOT GLUT_FOUND )
    message( "ERROR: GLUT not found" )
endif( NOT GLUT_FOUND )

# ~~ Include Directories ~~

# OpenGL
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

# cpp_helpers
include_directories( cpp_helpers ) 
set( CPP_HELP_CPP cpp_helpers/Cpp_Helpers.cpp )

# ___ END INCLUDE __________________________________________________________________________________________________________________________


# === CONFIGURE BUILD AND REPORT INFO ======================================================================================================

# ~ Executable Output ~
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR} ) # Output the executable to project toplevel

# ~ Report Status and Flags used ~
# https://cmake.org/Wiki/CMake_Useful_Variables
message( "" )
message( "~~~~ Library Directories ~~~~" )
message( "CMAKE_SOURCE_DIR               : ${CMAKE_SOURCE_DIR}" )
message( "CMAKE_RUNTIME_OUTPUT_DIRECTORY : ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}" )
message( "OPENGL_gl_LIBRARY              : ${OPENGL_gl_LIBRARY}" )
message( "OPENGL_glu_LIBRARY             : ${OPENGL_glu_LIBRARY}" )
message( "~~~~ Compiler Options ~~~~" )
message( "CMAKE_CXX_FLAGS : ${CMAKE_CXX_FLAGS}" )
message( "CMAKE_C_FLAGS   : ${CMAKE_C_FLAGS}" )
message( "" )
message( "Generating build files ..." )

# ___ END BUILD / INFO _____________________________________________________________________________________________________________________


# === LINK AND BUILD =======================================================================================================================

# ~~~ HOMEWORK 03 ~~~

add_executable( ${PROJECT_NAME} ${PROJECT_NAME}.cpp OGL_utils.cpp 
				${CPP_HELP_CPP} )

target_link_libraries( ${PROJECT_NAME} 
					   m
					   ${OPENGL_gl_LIBRARY}
					   ${OPENGL_glu_LIBRARY} 
					   ${GLUT_LIBRARY} ) 

# ___ END LINK BUILD _______________________________________________________________________________________________________________________

message( "CMake complete! , EXECUTABLE ---> ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}" )
message( "" )
