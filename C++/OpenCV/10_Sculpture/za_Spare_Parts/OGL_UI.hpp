#include "../SfM.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////
////////// OGL_Display.cpp /////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


/////////// GRAPHICS STRUCTS ///////////////////////////////////////////////////////////////////////

class Camera3D{ public:
	// Camera state goes here

	/// Members ///
	vec3d eyeLoc; // Camera location (world frame)
	vec3d lookPt; // Focus of camera (world frame)
	vec3d upVctr; // Direction of "up"0f};

    Camera3D();
    void look(); // ------------------------------------- Set camera position, target, and orientation
    void set_position( const vec3d& loc ); // ----------- Move the camera to `loc` and set view
    void set_target( const vec3d& target ); // ---------- Point the camera at `target` and set view
    void rotate_spherical( double theta, double phi ); // Rotate the eye about the target in spherical coordinates
    // Move the camera along the line of sight while preserving target and up, Enforce non-negative distance
    void zoom_spherical( double delDist );
};


enum VIEWMODE{ ORTHO, PERSP };


class OGL_window{ public:
	// Manage window things

    /// Singleton ///
    static OGL_window& make_window( int width , int height, string name = "WINDOW" );
	
	/// Members ///
	VIEWMODE CURRVIEW; // View type
	GLdouble dim; // ---- View scale
	GLdouble w2h; // ---- Width:Height Ratio
	GLdouble fov; // ---- Field Of View angle [deg]
    Camera3D cam; // ---- Camera
    
    /// Callback Pointers ///
    void (*draw_cb)(); // Drawing function
    void (*idle_cb)(); // Idle function

    OGL_window( VIEWMODE view = PERSP );

    void create( int width, int height, string name = "WINDOW" );
    void Project(); // ------- Set projection
    void OGL_frame_start(); // Do this before drawing anything
    void OGL_frame_end(); // - Do this after drawing everything, Flush and swap
    void set_callbacks( void (*draw_cb_)(), void (*idle_cb_)() = NULL );
    void run();

    void set_eye_target( const Point3d& eye, const Point3d& target );
};