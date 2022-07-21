# https://wiki.ogre3d.org/PyOgre+Tutorial1
 
import Ogre # ----- OGRE Core API
import Ogre.Bites # Frameworks & Templates

def main():
    # 0. Create an empty application
    app = Ogre.Bites.ApplicationContext( "02_scene" )
    app.initApp()
    
    # 1. Fetch application handle and add a Scene Manager
    root    = app.getRoot()
    scn_mgr = root.createSceneManager()
    win     = app.getRenderWindow()

    ##### Setting a Scene #####
    # https://wiki.ogre3d.org/PyOgre+Tutorial1#scene_Manager

    scn_mgr.setAmbientLight( [0.7, 0.7, 0.7] ) # NOTE: This affects objects NOT BG color!

    # DEBUG: Write available `SceneManager` members to a file so that we can follow the deprecated tutorial
    with open( "SceneManager-contents.txt", 'w' ) as f:
        f.writelines( [str(elem)+'\n' for elem in dir( scn_mgr )] )

    # N. Begin the rendering loop and keep window alive until GUI closes
    root.startRendering()


if __name__ == "__main__":
    main()