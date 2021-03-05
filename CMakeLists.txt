# TOPLEVEL CMAKELISTS

# ~ Project Setup ~
cmake_minimum_required( VERSION 3.0 )
project( PROJECTNAME )

# ~ Build Settings ~
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -lglut -lGLU -lGL")

# ~ Libraries ~
find_package( Eigen3 3.3 REQUIRED NO_MODULE )

find_package( OpenGL REQUIRED )
find_package( GLUT REQUIRED   )
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

# ~ Source Dirs ~
include_directories( 
    helpers #- Utility functions for OpenGL
    geometry # Meshes 'n Such
    STL # ---- STL Reader by Sebastian Reiter, https://github.com/sreiter/stl_reader
    tests # -- Library tests
) 


# ~~~ Defaults Test ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_executable( test01 tests/test01_defaults.cpp helpers/OGL_utils.cpp )
target_link_libraries( test01 Eigen3::Eigen ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )