##### INIT ######

# constants
const SCREEN_WIDTH    = 400;
const SCREEN_HEIGHT   = 300;
const CAMERA_DISTANCE = 5.0;
const TEXT_WIDTH      = 8;
const TEXT_HEIGHT     = 13;

# vars
global mouseLeftDown  = false;
global mouseRightDown = false;
global drawMode       = 0;
global screenWidth    = 0;
global screenHeight   = 0;
global mouseX         = 0;
global mouseY         = 0;
global cameraAngleX   = 0.0
global cameraAngleY   = 0.0;
global cameraDistance = 0.0;


# // vertex coords array for glDrawArrays() =====================================
# // A cube has 6 sides and each side has 2 triangles, therefore, a cube consists
# // of 36 vertices (6 sides * 2 tris * 3 vertices = 36 vertices). And, each
# // vertex is 3 components (x,y,z) of floats, therefore, the size of vertex
# // array is 108 floats (36 * 3 = 108).
global vertices = GLfloat[
     1, 1, 1,  -1, 1, 1,  -1,-1, 1,      #// v0-v1-v2 (front)
    -1,-1, 1,   1,-1, 1,   1, 1, 1,      #// v2-v3-v0

     1, 1, 1,   1,-1, 1,   1,-1,-1,      #// v0-v3-v4 (right)
     1,-1,-1,   1, 1,-1,   1, 1, 1,      #// v4-v5-v0

     1, 1, 1,   1, 1,-1,  -1, 1,-1,      #// v0-v5-v6 (top)
    -1, 1,-1,  -1, 1, 1,   1, 1, 1,      #// v6-v1-v0

    -1, 1, 1,  -1, 1,-1,  -1,-1,-1,      #// v1-v6-v7 (left)
    -1,-1,-1,  -1,-1, 1,  -1, 1, 1,      #// v7-v2-v1

    -1,-1,-1,   1,-1,-1,   1,-1, 1,      #// v7-v4-v3 (bottom)
     1,-1, 1,  -1,-1, 1,  -1,-1,-1,      #// v3-v2-v7

     1,-1,-1,  -1,-1,-1,  -1, 1,-1,      #// v4-v7-v6 (back)
    -1, 1,-1,   1, 1,-1,   1,-1,-1       #// v6-v5-v4
]   

# // normal array
global normals = GLfloat[
     0, 0, 1,   0, 0, 1,   0, 0, 1,      #// v0-v1-v2 (front)
     0, 0, 1,   0, 0, 1,   0, 0, 1,      #// v2-v3-v0

     1, 0, 0,   1, 0, 0,   1, 0, 0,      #// v0-v3-v4 (right)
     1, 0, 0,   1, 0, 0,   1, 0, 0,      #// v4-v5-v0

     0, 1, 0,   0, 1, 0,   0, 1, 0,      #// v0-v5-v6 (top)
     0, 1, 0,   0, 1, 0,   0, 1, 0,      #// v6-v1-v0

    -1, 0, 0,  -1, 0, 0,  -1, 0, 0,      #// v1-v6-v7 (left)
    -1, 0, 0,  -1, 0, 0,  -1, 0, 0,      #// v7-v2-v1

     0,-1, 0,   0,-1, 0,   0,-1, 0,      #// v7-v4-v3 (bottom)
     0,-1, 0,   0,-1, 0,   0,-1, 0,      #// v3-v2-v7

     0, 0,-1,   0, 0,-1,   0, 0,-1,      #// v4-v7-v6 (back)
     0, 0,-1,   0, 0,-1,   0, 0,-1       #// v6-v5-v4
] 
 

# // color array
global colors = GLfloat[
    1, 1, 1,   1, 1, 0,   1, 0, 0,      #// v0-v1-v2 (front)
    1, 0, 0,   1, 0, 1,   1, 1, 1,      #// v2-v3-v0

    1, 1, 1,   1, 0, 1,   0, 0, 1,      #// v0-v3-v4 (right)
    0, 0, 1,   0, 1, 1,   1, 1, 1,      #// v4-v5-v0

    1, 1, 1,   0, 1, 1,   0, 1, 0,      #// v0-v5-v6 (top)
    0, 1, 0,   1, 1, 0,   1, 1, 1,      #// v6-v1-v0

    1, 1, 0,   0, 1, 0,   0, 0, 0,      #// v1-v6-v7 (left)
    0, 0, 0,   1, 0, 0,   1, 1, 0,      #// v7-v2-v1

    0, 0, 0,   0, 0, 1,   1, 0, 1,      #// v7-v4-v3 (bottom)
    1, 0, 1,   1, 0, 0,   0, 0, 0,      #// v3-v2-v7

    0, 0, 1,   0, 0, 0,   0, 1, 0,      #// v4-v7-v6 (back)
    0, 1, 0,   0, 1, 1,   0, 0, 1       #// v6-v5-v4
]    

"""
Set the global vars to initial values
"""
function initSharedMem()
    # https://discourse.julialang.org/t/how-to-correctly-define-and-use-global-variables-in-the-module-in-julia/65720/2
    global mouseLeftDown, mouseRightDown, drawMode, screenWidth, screenHeight, mouseX, mouseY
    global cameraAngleX, cameraAngleY, cameraDistance
    screenWidth  = SCREEN_WIDTH;
    screenHeight = SCREEN_HEIGHT;

    mouseLeftDown = mouseRightDown = false;
    mouseX = mouseY = 0;

    cameraAngleX = cameraAngleY = 0.0;
    cameraDistance = CAMERA_DISTANCE;

    drawMode = 0; # 0:fill, 1: wireframe, 2:points

    return true;
end

function get_buffers( V, N, C )

    # https://github.com/Gnimuc/Videre/tree/master/OpenGL%204%20Tutorials
    # https://github.com/Gnimuc/Videre/blob/master/OpenGL%204%20Tutorials/03_vertex_buffer_objects/vbo.jl
    
    # Vertex Array Object
    vrtArrObj = glGenVertexArray()
    glBindVertexArray( vrtArrObj )

    vrtBufObj = glGenBuffer()
    glBindBuffer(GL_ARRAY_BUFFER, vrtBufObj)
    glBufferData(GL_ARRAY_BUFFER, sizeof(V), V, GL_STATIC_DRAW)
end


########## MAIN ####################################################################################

##### Init #####
initSharedMem()
# println( glInfo.isExtensionSupported("GL_ARB_vertex_buffer_object") ) # It is don't worry

println( cameraDistance )