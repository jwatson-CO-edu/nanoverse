# https://wiki.ogre3d.org/PyOgre+Tutorial1
 
import Ogre # ----- OGRE Core API
import Ogre.Bites # Frameworks & Templates

def main():
    # 0. Create an empty application
    app = Ogre.Bites.ApplicationContext( "01_window" )
    app.initApp()
    
    # 1. Fetch application handle and add a Scene Manager
    root    = app.getRoot()
    scn_mgr = root.createSceneManager()
    win     = app.getRenderWindow()

    # N. Begin the rendering loop and keep window alive until GUI closes
    root.startRendering()


if __name__ == "__main__":
    main()