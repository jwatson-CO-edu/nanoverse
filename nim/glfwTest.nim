import glfw

proc keyProc(window: GLFWWindow, key: int32, scancode: int32, action: int32, mods: int32): void {.cdecl.} =
  if key == GLFWKey.Escape and action == GLFWPress:
    window.setWindowShouldClose(true)

proc main() =
  assert glfwInit()

  let w: GLFWWindow = glfwCreateWindow(800, 600)
  if w == nil:
    quit(-1)

  discard w.setKeyCallback(keyProc)
  w.makeContextCurrent()

  while not w.windowShouldClose:
    glfwPollEvents()
    w.swapBuffers()

  w.destroyWindow()
  glfwTerminate()
  
main()