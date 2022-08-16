########## INIT ####################################################################################

##### Imports #####
from pythreejs import ( PerspectiveCamera, AmbientLight, Renderer, MeshBasicMaterial, Scene, 
                        OrbitControls, EdgesGeometry, LineBasicMaterial, LineSegments, Group )
from IPython.display import display


##### Constants #####
_CYL_RES        = 10
_COL_WHITE      = '#ffffff'
_COL_BLACK      = '#000000'
_COL_URGREY     = '#757d82'
_MAT_BSC_URGREY = MeshBasicMaterial( color = _COL_URGREY )
_MAT_BSC_WHITE  = MeshBasicMaterial( color = _COL_WHITE  )
_MAT_BSC_BLACK  = MeshBasicMaterial( color = _COL_BLACK  )


##### Env. Settings #####
np.set_printoptions( precision=3 )



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


def cylinder_w_edges( diameter, length, faceColor = _COL_WHITE, edgeColor = _COL_BLACK, edgeThick = 2, Nsegments = 10 ):
    cyl_geo = CylinderGeometry(
        radiusTop      = diameter / 2.0,
        radiusBottom   = diameter / 2.0,
        height         = length,
        radialSegments = _CYL_RES,
        heightSegments = _CYL_RES,
        openEnded      = False,
        thetaStart     = 0.0,
        thetaLength    = 2.0*np.pi
    )
    cyl_edg = EdgesGeometry( cyl_geo )
    cyl_mat = MeshBasicMaterial( color = faceColor )
    edg_mat = LineBasicMaterial( color = edgeColor, linewidth = edgeThick )
    cyl_msh = Mesh(
        geometry = cyl_geo,
        material = cyl_mat
    )
    cyl_lin = LineSegments(
        geometry = cyl_edg,
        material = edg_mat
    )
    rtnGrp = Group()
    rtnGrp.add( cyl_msh )
    rtnGrp.add( cyl_lin )
    return rtnGrp