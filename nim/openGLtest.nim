import opengl

# Create a context (GLFW recommended)

if glInit():
  echo "OpenGL loaded correctly."

glClearColor(0.68f, 1f, 0.34f, 1f)
glClear(GL_COLOR_BUFFER_BIT)

# You can load extensions

loadGL_ARB_gl_spirv()
if glSpecializeShaderARB == nil:
  echo "Extension not available"