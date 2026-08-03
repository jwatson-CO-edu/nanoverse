using OpenTK.Mathematics;
using OpenTK.Graphics.OpenGL4;


/// <summary>
/// Helper functions for this project
/// </summary>
namespace helpers {


/// <summary>
/// Working with `OpenTK.Mathematics.Vector3`
/// </summary>
public class MathVec3 {

    const float _EPSILON = 0.00001f;

    
    /// <summary>
    /// Are the vectors equal?
    /// </summary>
    public static bool Equal( Vector3 vec1 , Vector3 vec2 ){  return (vec1 - vec2).Length < _EPSILON;  }


    /// <summary>
    /// Get the angle between two R3 vectors , radians
    /// </summary>
    public static float AngleBetween( Vector3 vec1 , Vector3 vec2 ){
        float angle = (float) Math.Acos( Vector3.Dot( vec1.Normalized(), vec2.Normalized() ) ); // for now assume that there are no special cases
        if( float.IsNaN( angle ) ){
            if( Equal( vec1.Normalized(), vec2.Normalized() ) ){ return 0.0f; }
            else { return (float) Math.PI; }
        } else { return angle; }
    }


    /// <summary>
    /// `Quaternion.FromAxisAngle`, enforce normalized axis
    /// </summary>
    public static Quaternion AxisAngleQuat( Vector3 axis, float theta ){
        return Quaternion.FromAxisAngle( axis.Normalized(), theta );
    }


    public static Vector3 NoiseXY( Random rand, float scale = 1.0f ){
        return new Vector3( rand.NextSingle(), rand.NextSingle(), 0.0f ).Normalized() * scale;
    }


}


public class OGL{


    public static void CheckShader( int shader ){
        GL.GetShader( shader, ShaderParameter.CompileStatus, out int status );
        if (status == 0){  throw new InvalidOperationException("Shader compile error: " + GL.GetShaderInfoLog(shader));  }
    }


    public static int GetSimpleShaderProgram(){
        const string vertSrc = """
            #version 330 core
            layout(location = 0) in vec3 aPos;
            uniform mat4 uProjection;
            void main() {
                gl_Position = uProjection * vec4(aPos, 1.0);
            }
            """;

        const string fragSrc = """
            #version 330 core
            out vec4 FragColor;
            uniform vec4 uColor;
            void main() {
                FragColor = uColor;
            }
            """;

        int vs = GL.CreateShader( ShaderType.VertexShader );
        GL.ShaderSource( vs, vertSrc );
        GL.CompileShader( vs );
        CheckShader( vs );

        int fs = GL.CreateShader( ShaderType.FragmentShader );
        GL.ShaderSource( fs, fragSrc );
        GL.CompileShader( fs );
        CheckShader( fs );

        int prog = GL.CreateProgram();
        GL.AttachShader( prog, vs );
        GL.AttachShader( prog, fs );
        GL.LinkProgram( prog );
        GL.GetProgram( prog, GetProgramParameterName.LinkStatus, out int linkStatus );
        if (linkStatus == 0){
            throw new InvalidOperationException("Shader link error: " + GL.GetProgramInfoLog(prog));
        }
        GL.DeleteShader( vs );
        GL.DeleteShader( fs );
        return prog;
    }

    public static int[] GetSquareVAOandVBO( int NsqrPxls ){
        // --- Framebuffer: color texture + depth renderbuffer, rendered at supersample size ---
        int fbo = GL.GenFramebuffer();
        GL.BindFramebuffer( FramebufferTarget.Framebuffer, fbo );

        int colorTex = GL.GenTexture();
        GL.BindTexture( TextureTarget.Texture2D, colorTex );
        GL.TexImage2D( TextureTarget.Texture2D, 0, PixelInternalFormat.Rgba8, NsqrPxls, NsqrPxls, 0,
                       PixelFormat.Rgba, PixelType.UnsignedByte, IntPtr.Zero );
        GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int) TextureMinFilter.Linear );
        GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int) TextureMagFilter.Linear );
        GL.FramebufferTexture2D( FramebufferTarget.Framebuffer, 
                                 FramebufferAttachment.ColorAttachment0,
                                 TextureTarget.Texture2D, colorTex, 0 );

        int depthRbo = GL.GenRenderbuffer();
        GL.BindRenderbuffer( RenderbufferTarget.Renderbuffer, depthRbo );
        GL.RenderbufferStorage( RenderbufferTarget.Renderbuffer, RenderbufferStorage.DepthComponent24, NsqrPxls, NsqrPxls );
        GL.FramebufferRenderbuffer( FramebufferTarget.Framebuffer, 
                                    FramebufferAttachment.DepthAttachment,
                                    RenderbufferTarget.Renderbuffer, depthRbo );

        if( GL.CheckFramebufferStatus(FramebufferTarget.Framebuffer) != FramebufferErrorCode.FramebufferComplete ){
            throw new InvalidOperationException("Offscreen framebuffer is incomplete.");
        }

        GL.Viewport( 0, 0, NsqrPxls, NsqrPxls );
        GL.Enable( EnableCap.DepthTest );
        GL.Enable( EnableCap.Blend );
        GL.BlendFunc( BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha );

        // Parchment-ish background.
        GL.ClearColor( 0.965f, 0.945f, 0.89f, 1f );
        GL.Clear( ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit );

        // --- Upload sigil geometry (position-only triangles, normalized [0,1] xy + small z) ---
        int vao = GL.GenVertexArray();
        int vbo = GL.GenBuffer();
        GL.BindVertexArray( vao );
        GL.BindBuffer( BufferTarget.ArrayBuffer, vbo );
        int[] rtn = [vao, vbo];
        return rtn;
    }

}


}