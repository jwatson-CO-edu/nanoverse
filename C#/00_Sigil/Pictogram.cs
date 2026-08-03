using curve;
using helpers;
using OpenTK.Mathematics;
using OpenTK.Graphics.OpenGL4;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.Formats.Jpeg;
using SixLabors.ImageSharp.PixelFormats;
using SixLabors.ImageSharp.Processing;

namespace sigil {

/// <summary>
/// The actual Sigil
/// </summary>
public class Pictogram ( int scale = 1024, float thickness = 25.0f ) {

    /// Constants ///
    public const int    _MAX_STROKES = 64; 
    public const float  _GAP_FACTOR  = 0.1f; 
    public const float  _LIN_FACTOR  = 0.5f; 
    public const float  _LAYER_STEP  = 1f/64f; 

    /// Members ///
    public List<Stroke> strokes /**/ = [];
    public float /*--*/ thick /*--*/ = thickness;
    public float /*--*/ scale /*--*/ = scale;
    public float /*--*/ gapScale     = scale * _GAP_FACTOR;
    public float /*--*/ linScale     = scale * _LIN_FACTOR;
    public Vector3 /**/ _Z_DIR /*-*/ = new(0,0,1);



    public void Generate( int maxStrokes = _MAX_STROKES, float breakProb = 1.0f / _MAX_STROKES ){
        int /*--*/ maxStrk  = maxStrokes;
        int /*--*/ count    = 0;
        Random     random   = new();
        Parametric lastCurv = new DummyCurve();
        Parametric currCurv;
        int /*--*/ loc, type; 
        float /**/ tBgn, offset;
        Vector3    vBgn, bTan, bCrv, bPnt, bDir, endPnt, midPnt, P1, P2;

        strokes.Capacity = maxStrk;
        
        while( count < maxStrk ){

            ///// Roll for start location, Select {0, 1, t} /////
            loc = random.Next(3);
            tBgn = loc switch{
                0 => 0.0f,
                1 => 1.0f,
                2 => random.NextSingle(),
                _ => throw new InvalidDataException( $"{loc} was NOT a valid choice" ),
            };

            // FIXME: THIS FUNCTION ONLY CONNECTS CURVES SEQUENTIALLY, RANDOMLY WOULD BE MORE FUN

            ///// Roll for start orientation, Select {Tangent, Curvature, Oblique,} /////
            loc    = random.Next(3);
            offset = random.Next(2) * random.NextSingle() * gapScale; // Zero -or- Gap
            bDir   = MathVec3.NoiseXY( random ); 
            if( lastCurv is DummyCurve ){
                vBgn = new Vector3( scale/2.0f, scale/2.0f, 0.0f );
                bTan = MathVec3.NoiseXY( random );
                bCrv = MathVec3.NoiseXY( random );
            }else{
                vBgn = lastCurv.Val( tBgn );
                bTan = lastCurv.Tan( tBgn );
                bCrv = lastCurv.Crv( tBgn );
            }
            bPnt = loc switch{
                0 => vBgn + bTan * offset,
                1 => vBgn + bCrv * offset,
                2 => vBgn + bDir * offset,
                _ => throw new InvalidDataException($"{loc} was NOT a valid choice"),
            };
            bDir = loc switch{
                0 => bTan.Normalized(),
                1 => bCrv.Normalized(),
                2 => bDir,
                _ => throw new InvalidDataException($"{loc} was NOT a valid choice"),
            };

            ///// Roll for Stroke Type /////
            type   = random.Next(4);
            offset = linScale * random.NextSingle();
            switch( type ){
                
                /// Line Segment ///
                case 0:
                    currCurv = new Line.Segment( bPnt, bPnt + bDir * offset );
                    break;
                
                /// Circle ///
                case 1:
                    currCurv = new Ellipse.Circle( bPnt + bDir * offset, _Z_DIR, offset );
                    break;
                
                /// Quad Bezier ///
                case 2:
                    endPnt   = bPnt + bDir * offset + MathVec3.NoiseXY( random, linScale * 0.25f );
                    midPnt   = (bPnt + endPnt)/2.0f + MathVec3.NoiseXY( random, linScale * 0.125f );
                    currCurv = new Bezier.Quad( bPnt, midPnt, endPnt );
                    break;
                
                /// Cube Bezier ///
                case 3:
                    endPnt   = bPnt + bDir * offset   + MathVec3.NoiseXY( random, linScale * 0.250f );
                    midPnt   = (bPnt + endPnt)/2.0f   + MathVec3.NoiseXY( random, linScale * 0.125f );
                    P1 /*-*/ = (bPnt + midPnt)/2.0f   + MathVec3.NoiseXY( random, linScale * 0.125f );
                    P2 /*-*/ = (midPnt + endPnt)/2.0f + MathVec3.NoiseXY( random, linScale * 0.125f );
                    currCurv = new Bezier.Cubic( bPnt, P1, P2, endPnt );
                    break;
                
                /// Should Not Happen ///
                default:
                    throw new InvalidDataException( $"{type} was NOT a valid choice" );

            }

            Stroke /*------*/ nuStroke = new( currCurv, thick );
            // int /*---------*/ Nintersect;
            // int /*---------*/ interChoice;
            // List<List<float>> intersections;

            
            // // For each existing stroke
            // for( int i = 0; i < count; ++i ){

            //     // Disre
            //     if( nuStroke.HasNeighbor( strokes[i] ) ){  continue;  }

            //     // Scan for intersections
            //     intersections = nuStroke.GetIntersections( strokes[i], 1f/128f, thick );
            //     Nintersect    = intersections[0].Count / 2;
            //     // For each intersection  
            //     for( int j = 0; j < Nintersect; ++j ){
            //         interChoice = random.Next(3);
            //         switch( interChoice ){
            //             /// Cross ///
            //             case 0:
            //                 // FIXME: REMOVE UNDERPAINT FROM THIS SPAN
            //                 break;
            //             /// Above ///
            //             case 1:
            //                 // FIXME: BUMP OFFSET UP
            //                 break;
            //             /// Below ///
            //             case 2:
            //                 // FIXME: BUMP OFFSET DOWN
            //                 break;
            //             /// This should NOT happen! ///
            //             default:
            //                 throw new InvalidDataException( $"{interChoice} was NOT a valid choice" );
            //         }
            //     }
            // }

            
            
            if( random.NextSingle() < breakProb ){  break;  } // Roll for break
            count++;
            lastCurv = strokes[ random.Next( count ) ].curve;
        }

        // FIXME: FOR EACH STROKE
            // CREATE GEO
                // STROKE
                // UNDERSTROKE
        foreach( Stroke strk in strokes ){
            strk.ReserveGeo();
            strk.BuildGeo();
        }
    }

}


/// <summary>
/// Renders triangles to a JPG
/// </summary>
public class Renderer{


    /// <summary>
    /// Renders triangles to a JPG
    /// </summary>
    public static void CheckShader( int shader ){
        GL.GetShader( shader, ShaderParameter.CompileStatus, out int status );
        if (status == 0){  throw new InvalidOperationException("Shader compile error: " + GL.GetShaderInfoLog(shader));  }
    }


    /// <summary>
    /// Simplest shader program with no lighting
    /// </summary>
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


    public int vao; // ---- Vertex Attribute Object
    public int vbo; // ---- Vertex Buffer Object
    public int fbo; // ---- Frame Buffer Object
    public int colorTex; // Generated Texture Location
    public int depthRbo; // Depth Render Buffer Object
    public int program; //- Shader Program: Vertex + Fragment


    /// <summary>
    /// Allocate buffers needed for rendering
    /// </summary>
    public void GetSquareBuffers( int NsqrPxls ){
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

        if( GL.CheckFramebufferStatus( FramebufferTarget.Framebuffer ) != FramebufferErrorCode.FramebufferComplete ){
            throw new InvalidOperationException( "Offscreen framebuffer is incomplete." );
        }

        GL.Viewport( 0, 0, NsqrPxls, NsqrPxls );
        GL.Enable( EnableCap.DepthTest );
        GL.Enable( EnableCap.Blend );
        GL.BlendFunc( BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha );

        // Parchment-ish background.
        GL.ClearColor( 0.965f, 0.945f, 0.89f, 1f );
        GL.Clear( ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit );

        // --- Upload sigil geometry (position-only triangles, normalized [0,1] xy + small z) ---
        vao = GL.GenVertexArray();
        vbo = GL.GenBuffer();
        GL.BindVertexArray( vao );
        GL.BindBuffer( BufferTarget.ArrayBuffer, vbo );
    }


    /// <summary>
    /// Render triangles to JPG at specified `_outputPath`
    /// </summary>
    public void TriangleList2JPG( List<Tri> _triangles, int NsqrPxls, string _outputPath ){

        float[] data = Tri.Triangles2Arr( _triangles );

        GL.BufferData( BufferTarget.ArrayBuffer, 
                       data.Length * sizeof(float), 
                       data, BufferUsageHint.StaticDraw );
        GL.VertexAttribPointer( 0, 3, VertexAttribPointerType.Float, false, 3 * sizeof(float), 0 );
        GL.EnableVertexAttribArray(0);

        GL.UseProgram( program );
        var proj = Matrix4.CreateOrthographicOffCenter( 0f, 1f, 0f, 1f, -1f, 1f );
        GL.UniformMatrix4( GL.GetUniformLocation( program, "uProjection" ), false, ref proj );
        GL.Uniform4(
            GL.GetUniformLocation( program, "uColor" ), 
            new Vector4( 0.09f, 0.08f, 0.10f, 1f )
        );

        GL.DrawArrays( PrimitiveType.Triangles, 0, _triangles.Count * 3 );
        GL.Flush();

        // --- Read back and save ---
        byte[] pixels = new byte[ NsqrPxls * NsqrPxls * 4 ];
        GL.ReadPixels( 0, 0, NsqrPxls, NsqrPxls, PixelFormat.Rgba, PixelType.UnsignedByte, pixels );

        using var image = Image.LoadPixelData<Rgba32>( pixels, NsqrPxls, NsqrPxls );
        image.Mutate(ctx => ctx.Flip( FlipMode.Vertical ) ); // GL origin is bottom-left
        image.Mutate(ctx => ctx.Resize( new ResizeOptions{
            Size = new Size( NsqrPxls, NsqrPxls ),
            Sampler = KnownResamplers.Lanczos3, // supersample downscale acts as anti-aliasing
        }));
        image.Save( _outputPath, new JpegEncoder { Quality = 92 } );

        GL.DeleteVertexArray( vao );
        GL.DeleteBuffer( vbo );
        GL.DeleteFramebuffer( fbo );
        GL.DeleteTexture( colorTex );
        GL.DeleteRenderbuffer( depthRbo );
        GL.DeleteProgram( program );
    }
}

}