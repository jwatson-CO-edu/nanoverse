import GLFW
using ModernGL
include("util.jl")


##### Create Window #####

GLFW.Init()
# OS X-specific GLFW hints to initialize the correct version of OpenGL
wh = 600
# Create a windowed mode window and its OpenGL context
window = GLFW.CreateWindow(wh, wh, "OpenGL Example")
# Make the window's context current
GLFW.MakeContextCurrent(window)
GLFW.ShowWindow(window)
GLFW.SetWindowSize(window, wh, wh) # Seems to be necessary to guarantee that window > 0

glViewport(0, 0, wh, wh)

println(createcontextinfo())


##### Create Tetrahedron #####
# FIXME: START HERE
# https://math.stackexchange.com/questions/562741/what-are-the-coordinates-of-the-vertices-of-a-regular-tetrahedron-relative-to-i
