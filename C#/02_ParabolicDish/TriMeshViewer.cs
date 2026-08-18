using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;

namespace geo3d {


/// <summary>
/// Opens a window and renders a `List<Tri>;` mesh with flat-shaded Phong lighting.
/// Call this from your `Main`/entry point; it blocks until the window is closed.
/// </summary>
public static class TriMeshViewer {

    /// <summary>
    /// Display `mesh` in an OpenGL window with a CAD-ball orbit camera.
    /// </summary>
    public static void Show( List<Tri> mesh, string title = "Tri Mesh Viewer", int width = 1280, int height = 800 ){

        var gwSettings = GameWindowSettings.Default;
        gwSettings.UpdateFrequency = 60.0;

        var nwSettings = new NativeWindowSettings(){
            ClientSize = new Vector2i( width, height ),
            Title      = title,
            APIVersion = new Version( 3, 3 ), // Core 3.3 is enough for this shader
            Profile    = ContextProfile.Core,
        };

        using var window = new TriMeshWindow( mesh, gwSettings, nwSettings );
        window.Run();
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////
////////// GAME WINDOW /////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

internal sealed class TriMeshWindow : GameWindow {

    // --- Mesh data ---
    private readonly List<Tri> mesh;
    private int vao, vbo, vertCount;

    // --- Shader ---
    private int program;
    private int uModel, uView, uProj, uLightPos, uViewPos, uObjColor, uLightColor, uAmbient;

    // --- Camera ---
    private CadBallCamera camera = null!;

    // --- Colors ---
    private static readonly Vector3 ObjectColor     = new( 0.15f, 0.35f, 0.85f ); // blue
    private static readonly Vector3 LightColor /**/ = new( 1.0f,  1.0f,  1.0f  ); // white
    private const float /*-------*/ AmbientStrength = 0.55f; // generous ambient

    public TriMeshWindow( List<Tri> mesh_, GameWindowSettings gwSettings, NativeWindowSettings nwSettings )
        : base( gwSettings, nwSettings ){
        mesh = mesh_;
    }


    // ---- LOAD ---------------------------------------------------------------------------------

    protected override void OnLoad(){
        base.OnLoad();

        GL.ClearColor( 0.0f, 0.0f, 0.0f, 1.0f ); // black clear color
        GL.Enable( EnableCap.DepthTest );
        GL.Enable( EnableCap.CullFace ); // comment out if source meshes aren't consistently wound
        GL.CullFace( TriangleFace.Back );

        BuildMeshBuffers();
        BuildShader();

        Vector3[] bbox   = Tri.MeshBBox( mesh );
        Vector3   center = ( bbox[0] + bbox[1] ) * 0.5f;
        float     radius = ( bbox[1] - bbox[0] ).Length * 0.5f;
        if( radius < 0.001f ){  radius = 1.0f;  }

        camera = new CadBallCamera( center, distance: radius * 3.0f, clientSize: ClientSize );

        CursorState = CursorState.Normal;
    }


    /// <summary>
    /// Upload position + face-normal interleaved vertex data (flat-shaded: every vertex
    /// of a triangle shares that triangle's normal, so smooth interpolation reproduces
    /// a faceted look without needing a `flat` qualifier).
    /// </summary>
    private void BuildMeshBuffers(){
        vertCount = mesh.Count * 3;
        float[] data = new float[ vertCount * 6 ]; // xyz + nxnynz per vertex

        int i = 0;
        foreach( Tri tri in mesh ){
            Vector3 ab     = tri.V1() - tri.V0();
            Vector3 ac     = tri.V2() - tri.V0();
            Vector3 normal = Vector3.Cross( ab, ac );
            normal = normal.LengthSquared > 1e-12f ? normal.Normalized() : Vector3.UnitY;

            for( int v = 0; v < 3; ++v ){
                Vector3 p = tri[v];
                int off = i * 6;
                data[off + 0] = p.X;
                data[off + 1] = p.Y;
                data[off + 2] = p.Z;
                data[off + 3] = normal.X;
                data[off + 4] = normal.Y;
                data[off + 5] = normal.Z;
                ++i;
            }
        }

        vao = GL.GenVertexArray();
        vbo = GL.GenBuffer();

        GL.BindVertexArray( vao );
        GL.BindBuffer( BufferTarget.ArrayBuffer, vbo );
        GL.BufferData( BufferTarget.ArrayBuffer, data.Length * sizeof(float), data, BufferUsageHint.StaticDraw );

        int stride = 6 * sizeof(float);
        GL.VertexAttribPointer( 0, 3, VertexAttribPointerType.Float, false, stride, 0 );
        GL.EnableVertexAttribArray( 0 );
        GL.VertexAttribPointer( 1, 3, VertexAttribPointerType.Float, false, stride, 3 * sizeof(float) );
        GL.EnableVertexAttribArray( 1 );

        GL.BindVertexArray( 0 );
    }


    private void BuildShader(){
        const string vertSrc = """
            #version 330 core
            layout(location = 0) in vec3 aPos;
            layout(location = 1) in vec3 aNormal;

            uniform mat4 uModel;
            uniform mat4 uView;
            uniform mat4 uProj;

            out vec3 vFragPos;
            out vec3 vNormal;

            void main(){
                vFragPos    = vec3( uModel * vec4( aPos, 1.0 ) );
                vNormal     = mat3( transpose( inverse( uModel ) ) ) * aNormal;
                gl_Position = uProj * uView * vec4( vFragPos, 1.0 );
            }
            """;

        const string fragSrc = """
            #version 330 core
            in vec3 vFragPos;
            in vec3 vNormal;
            out vec4 FragColor;

            uniform vec3  uLightPos;
            uniform vec3  uViewPos;
            uniform vec3  uObjectColor;
            uniform vec3  uLightColor;
            uniform float uAmbientStrength;

            void main(){
                vec3 norm     = normalize( vNormal );
                vec3 lightDir = normalize( uLightPos - vFragPos );

                float diff    = max( dot( norm, lightDir ), 0.0 );

                vec3 viewDir    = normalize( uViewPos - vFragPos );
                vec3 reflectDir = reflect( -lightDir, norm );
                float spec      = pow( max( dot( viewDir, reflectDir ), 0.0 ), 32.0 );

                vec3 ambient  = uAmbientStrength * uLightColor;
                vec3 diffuse  = diff * uLightColor;
                vec3 specular = 0.25 * spec * uLightColor;

                vec3 result = ( ambient + diffuse + specular ) * uObjectColor;
                FragColor   = vec4( result, 1.0 );
            }
            """;

        int vert = CompileShader( ShaderType.VertexShader, vertSrc );
        int frag = CompileShader( ShaderType.FragmentShader, fragSrc );

        program = GL.CreateProgram();
        GL.AttachShader( program, vert );
        GL.AttachShader( program, frag );
        GL.LinkProgram( program );
        GL.GetProgram( program, GetProgramParameterName.LinkStatus, out int linked );
        if( linked == 0 ){
            string log = GL.GetProgramInfoLog( program );
            throw new Exception( $"Shader link error: {log}" );
        }

        GL.DeleteShader( vert );
        GL.DeleteShader( frag );

        uModel      = GL.GetUniformLocation( program, "uModel" );
        uView       = GL.GetUniformLocation( program, "uView" );
        uProj       = GL.GetUniformLocation( program, "uProj" );
        uLightPos   = GL.GetUniformLocation( program, "uLightPos" );
        uViewPos    = GL.GetUniformLocation( program, "uViewPos" );
        uObjColor   = GL.GetUniformLocation( program, "uObjectColor" );
        uLightColor = GL.GetUniformLocation( program, "uLightColor" );
        uAmbient    = GL.GetUniformLocation( program, "uAmbientStrength" );
    }


    private static int CompileShader( ShaderType type, string src ){
        int handle = GL.CreateShader( type );
        GL.ShaderSource( handle, src );
        GL.CompileShader( handle );
        GL.GetShader( handle, ShaderParameter.CompileStatus, out int compiled );
        if( compiled == 0 ){
            string log = GL.GetShaderInfoLog( handle );
            throw new Exception( $"{type} compile error: {log}" );
        }
        return handle;
    }


    // ---- RENDER --------------------------------------------------------------------------------

    protected override void OnRenderFrame( FrameEventArgs args ){
        base.OnRenderFrame( args );

        GL.Clear( ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit );
        GL.UseProgram( program );

        Matrix4 model = Matrix4.Identity;
        Matrix4 view  = camera.GetViewMatrix();
        Matrix4 proj  = Matrix4.CreatePerspectiveFieldOfView(
                            MathHelper.DegreesToRadians( 45.0f ),
                            ClientSize.X / (float) MathF.Max( ClientSize.Y, 1 ),
                            0.01f, 1000.0f );

        GL.UniformMatrix4( uModel, false, ref model );
        GL.UniformMatrix4( uView,  false, ref view  );
        GL.UniformMatrix4( uProj,  false, ref proj  );

        // Light sits behind the camera (further back along the eye->target axis), gentle & white.
        Vector3 eye       = camera.EyePosition;
        Vector3 behindDir = ( eye - camera.Target ).Normalized();
        Vector3 lightPos  = eye + behindDir * ( camera.Distance * 0.5f + 2.0f );

        GL.Uniform3( uLightPos,   lightPos );
        GL.Uniform3( uViewPos,    eye );
        GL.Uniform3( uObjColor,   ObjectColor );
        GL.Uniform3( uLightColor, LightColor );
        GL.Uniform1( uAmbient,    AmbientStrength );

        GL.BindVertexArray( vao );
        GL.DrawArrays( PrimitiveType.Triangles, 0, vertCount );
        GL.BindVertexArray( 0 );

        SwapBuffers();
    }


    protected override void OnResize( ResizeEventArgs e ){
        base.OnResize( e );
        GL.Viewport( 0, 0, e.Width, e.Height );
        camera?.OnResize( ClientSize );
    }


    // ---- CAD-BALL MOUSE NAVIGATION --------------------------------------------------------------
    // Left-drag  : arcball rotate
    // Right-drag : pan
    // Scroll     : zoom (dolly)

    protected override void OnMouseDown( MouseButtonEventArgs e ){
        base.OnMouseDown( e );
        if( e.Button == MouseButton.Left  ){  camera.BeginRotate( MousePosition );  }
        if( e.Button == MouseButton.Right ){  camera.BeginPan( MousePosition );  }
    }

    protected override void OnMouseUp( MouseButtonEventArgs e ){
        base.OnMouseUp( e );
        if( e.Button == MouseButton.Left  ){  camera.EndRotate();  }
        if( e.Button == MouseButton.Right ){  camera.EndPan();  }
    }

    protected override void OnMouseMove( MouseMoveEventArgs e ){
        base.OnMouseMove( e );
        camera.OnMouseMove( MousePosition );
    }

    protected override void OnMouseWheel( MouseWheelEventArgs e ){
        base.OnMouseWheel( e );
        camera.OnScroll( e.OffsetY );
    }


    protected override void OnUnload(){
        GL.DeleteBuffer( vbo );
        GL.DeleteVertexArray( vao );
        GL.DeleteProgram( program );
        base.OnUnload();
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////
////////// CAD-BALL (ARCBALL) CAMERA ///////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

/// <summary>
/// Orbit camera driven by a virtual-trackball ("CAD ball") mapping: mouse position is
/// projected onto a unit hemisphere centered on the viewport, and drag deltas on that
/// sphere become an arbitrary-axis rotation of the camera around a fixed target point.
/// This gives the free, gimbal-lock-free rotation feel of CAD packages (SolidWorks/
/// Fusion-style "rotate"), rather than a constrained yaw/pitch orbit.
/// </summary>
internal sealed class CadBallCamera {

    public Vector3 Target   { get; private set; }
    public float   Distance { get; private set; }

    private Quaternion _rotation = Quaternion.Identity; // maps canonical camera axes -> world axes
    private Vector2i   _clientSize;

    private bool     _rotating;
    private Vector3  _lastBallPoint;

    private bool    _panning;
    private Vector2 _lastPanPixel;

    public CadBallCamera( Vector3 target, float distance, Vector2i clientSize ){
        Target      = target;
        Distance    = MathF.Max( distance, 0.01f );
        _clientSize = clientSize;
    }

    public Vector3 EyePosition => Target + ( _rotation * Vector3.UnitZ ) * Distance;
    private Vector3 Up          => _rotation * Vector3.UnitY;
    private Vector3 Right       => _rotation * Vector3.UnitX;

    public void OnResize( Vector2i clientSize ){  _clientSize = clientSize;  }

    public Matrix4 GetViewMatrix() => Matrix4.LookAt( EyePosition, Target, Up );


    // ---- Rotate (left-drag) ----

    public void BeginRotate( Vector2 mousePixel ){
        _rotating      = true;
        _lastBallPoint = ProjectToBall( mousePixel );
    }

    public void EndRotate(){  _rotating = false;  }

    public void OnMouseMove( Vector2 mousePixel ){
        if( _rotating ){
            Vector3 curPoint = ProjectToBall( mousePixel );

            Vector3 axisCam = Vector3.Cross( _lastBallPoint, curPoint );
            float   dotVal  = Math.Clamp( Vector3.Dot( _lastBallPoint, curPoint ), -1.0f, 1.0f );
            float   angle   = MathF.Acos( dotVal );

            if( axisCam.LengthSquared > 1e-10f && angle > 1e-6f ){
                // axisCam is expressed in the camera's local frame (x right, y up, z toward eye);
                // rotate it into world space with the camera's current orientation.
                Vector3 axisWorld = _rotation * axisCam.Normalized();
                // Negative angle: dragging the ball should carry the *scene* with the cursor,
                // which means rotating the camera the opposite way around that axis.
                Quaternion delta = Quaternion.FromAxisAngle( axisWorld, -angle );
                _rotation = Quaternion.Normalize( delta * _rotation );
            }

            _lastBallPoint = curPoint;
        }

        if( _panning ){
            Vector2 delta = mousePixel - _lastPanPixel;
            float   scale = Distance * 0.0015f; // pan speed scales with zoom level
            Target -= Right * delta.X * scale;
            Target += Up    * delta.Y * scale;
            _lastPanPixel = mousePixel;
        }
    }

    /// <summary>
    /// Map a pixel coordinate to a point on the unit hemisphere (camera-local x,y,z; z toward viewer).
    /// </summary>
    private Vector3 ProjectToBall( Vector2 mousePixel ){
        float w = MathF.Max( _clientSize.X, 1 );
        float h = MathF.Max( _clientSize.Y, 1 );

        // NDC in [-1,1], y flipped so up is positive
        float nx = ( 2.0f * mousePixel.X / w ) - 1.0f;
        float ny = 1.0f - ( 2.0f * mousePixel.Y / h );

        float d2 = nx * nx + ny * ny;
        if( d2 <= 1.0f ){  return new Vector3( nx, ny, MathF.Sqrt( 1.0f - d2 ) );  }
        return new Vector3( nx, ny, 0.0f ).Normalized();
    }


    // ---- Pan (right-drag) ----

    public void BeginPan( Vector2 mousePixel ){
        _panning      = true;
        _lastPanPixel = mousePixel;
    }

    public void EndPan(){  _panning = false;  }


    // ---- Zoom (scroll) ----

    public void OnScroll( float offsetY ){
        float factor = MathF.Pow( 0.9f, offsetY );
        Distance = Math.Clamp( Distance * factor, 0.05f, 10000.0f );
    }
}

}
