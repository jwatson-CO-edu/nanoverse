########## INIT ####################################################################################

##### Imports #####

from pythreejs import ( PerspectiveCamera, AmbientLight, Renderer, MeshBasicMaterial, Scene, 
                        OrbitControls )
from IPython.display import display


##### Constants #####
_CYL_RES        = 10
_COL_WHITE      = '#ffffff'
_COL_BLACK      = '#000000'
_COL_URGREY     = '#757d82'
_MAT_BSC_URGREY = MeshBasicMaterial( color = _COL_URGREY )
_MAT_BSC_WHITE  = MeshBasicMaterial( color = _COL_WHITE  )
_MAT_BSC_BLACK  = MeshBasicMaterial( color = _COL_BLACK  )

########## RENDERING UTILITIES #####################################################################

def get_renderer_for_obj_list( objLst, camPos = [1.0,1.0,1.0], lookPos = [0,0,0], bgColor = _COL_BLACK, width = 600, height = 600 ):
    camera = PerspectiveCamera(
        position = camPos,
        lookAt   = lookPos
    )
    
    renderList = [
        AmbientLight(
            color = '#FFFFFF'
        ),
    ]
    renderList.extend( objLst )

    scene = Scene(
        background         = bgColor, 
        background_opacity = 1.0,
        children           = renderList
    )

    renderer = Renderer(
        camera = camera,
        scene = scene,
        controls = [
            OrbitControls(
                controlling = camera
            )
        ],
        width  = width, 
        height = height
    )
    
    return renderer