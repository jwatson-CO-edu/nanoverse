using curve;
using helpers;

using OpenTK.Mathematics;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;

using SixLabors.ImageSharp;
using SixLabors.ImageSharp.Formats.Jpeg;
using SixLabors.ImageSharp.PixelFormats;
using SixLabors.ImageSharp.Processing;

namespace sigil {

/// <summary>
/// The actual Sigil
/// </summary>
public class Pictogram ( float scale = 0.75f, float thickness = 0.2f ) {

    /// Constants ///
    public const int    _MAX_STROKES = 16; // 32; //64; 
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


    /// <summary>
    /// Generate the actual Sigil
    /// </summary>
    public void Generate( int maxStrokes = _MAX_STROKES, float breakProb = 1.0f / (4.0f*_MAX_STROKES), float gridProb = 0.75f ){
        int /*--*/ maxStrk  = maxStrokes;
        int /*--*/ count    = 0;
        Random     random   = new();
        Parametric lastCurv = new DummyCurve();
        Parametric currCurv;
        int /*--*/ loc, type; 
        float /**/ tBgn, offset, factor;
        Vector3    vBgn, bTan, bCrv, bPnt, bDir, endPnt, midPnt, P1, P2;

        strokes.Capacity = maxStrk;
        
        while( count < maxStrk ){

            Console.WriteLine( $"Iteration {count+1}" );

            ///// Roll for start location, Select {0, 1, t} /////
            loc = random.Next(3);
            tBgn = loc switch{
                0 => 0.0f,
                1 => 1.0f,
                2 => random.NextSingle(),
                _ => throw new InvalidDataException( $"{loc} was NOT a valid choice" ),
            };

            // FIXME: THIS FUNCTION ONLY CONNECTS CURVES SEQUENTIALLY, RANDOMLY WOULD BE MORE FUN

            Console.WriteLine( $"\tRoll orientation ..." );

            ///// Roll for start orientation, Select {Tangent, Curvature, Oblique,} /////
            loc    = random.Next(3);
            offset = random.Next(2) * random.NextSingle() * gapScale; // Zero -or- Gap
            bDir   = MathVec3.NoiseXY( random ); 
            if( lastCurv is DummyCurve ){
                // vBgn = new Vector3( scale/2.0f, scale/2.0f, 0.0f );
                vBgn = new Vector3( 0.0f, 0.0f, 0.0f );
                bTan = MathVec3.NoiseXY( random );
                bCrv = MathVec3.NoiseXY( random );
            }else{
                vBgn = lastCurv.Val( tBgn );
                bTan = lastCurv.Tan( tBgn ).Normalized();
                bCrv = lastCurv.Crv( tBgn ).Normalized();
            }
            bPnt = loc switch{
                0 => vBgn + bTan * offset,
                1 => vBgn + bCrv * offset,
                2 => vBgn + bDir * offset,
                _ => throw new InvalidDataException($"{loc} was NOT a valid choice"),
            };

            if( (lastCurv is Bezier.Cubic) || (lastCurv is Bezier.Quad) ){ bDir = bTan; }else{
                bDir = loc switch{
                    0 => bTan,
                    1 => bCrv,
                    2 => bDir,
                    _ => throw new InvalidDataException($"{loc} was NOT a valid choice"),
                };
            }

            Console.WriteLine( $"\tRoll stroke type ..." );

            ///// Roll for Stroke Type /////
             
            type   = random.Next(4);
            offset = 4*thick + linScale * random.NextSingle();
            
            switch( type ){
                
                /// Line Segment ///
                case 0:
                    currCurv = new Line.Segment( bPnt, bPnt + bDir * offset );
                    break;
                
                /// Circle ///
                case 1:
                    factor   = 0.25f;
                    factor   = offset * (factor + factor*random.NextSingle());
                    currCurv = new Ellipse.Circle( bPnt + bDir * factor, _Z_DIR, factor );
                    break;
                
                /// Quad Bezier ///
                case 2:
                    midPnt   = bPnt + bDir * 0.5f*offset; 
                    endPnt   = midPnt + MathVec3.NoiseXY( random, 0.5f*offset );
                    currCurv = new Bezier.Quad( bPnt, midPnt, endPnt );
                    break;
                
                /// Cube Bezier ///
                case 3:
                    midPnt   = bPnt + bDir * 0.5f*offset;
                    P1 /*-*/ = bPnt + bDir * (0.5f*offset-0.25f*offset*random.NextSingle());
                    endPnt   = midPnt + MathVec3.NoiseXY( random, offset );
                    P2 /*-*/ = (midPnt + endPnt)/2.0f + MathVec3.NoiseXY( random, linScale * 0.0625f );
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
            strokes.Add( nuStroke );
            count++;
            lastCurv = strokes[ random.Next( count ) ].curve;
        }

        foreach( Stroke strk in strokes ){
            Console.WriteLine( $"Reserve geo ..." );
            strk.ReserveGeo();
            Console.WriteLine( $"Build geo ..." );
            strk.BuildGeo();
        }
    }


    /// <summary>
    /// Count all triangles the actual Sigil
    /// </summary>
    public int CountAllTriangles(){
        int rtnTri = 0;
        foreach( Stroke strk in strokes ){
            rtnTri += strk.geo.Count;
        }
        return rtnTri;
    }


    /// <summary>
    /// Get all triangles the actual Sigil
    /// </summary>
    public List<Tri> GetAllTriangles(){
        List<Tri> rtnTri = [];
        rtnTri.Capacity = CountAllTriangles();
        foreach( Stroke strk in strokes ){
            rtnTri.AddRange( strk.geo );
        }
        return rtnTri;
    }


    /// <summary>
    /// Area Centroid of Sigil
    /// </summary>
    public Vector3 Centroid(){
        float   area_i;
        float   totArea  = 0.0f;
        Vector3 centroid = new(0,0,0);
        foreach( Stroke strk in strokes ){
            area_i    = strk.Area();
            totArea  += area_i;
            centroid += strk.Centroid() * area_i;
        }
        return centroid / totArea;
    }
    

    /// <summary>
    /// Shift entire Sigil
    /// </summary>
    public void Shift( Vector3 offset ){  foreach( Stroke strk in strokes ){  strk.Shift( offset );  }  }


    public void ShiftToCenter(){  Shift( -Centroid() );  }

}


/// <summary>
/// A hidden GameWindow whose only job is to own a valid GL context long enough to render
/// one sigil into an offscreen framebuffer and save it. Using GameWindow (rather than a
/// bare context) means OpenTK handles context creation/binding-loading for us in the
/// normal, well-tested way; OnLoad fires once bindings are ready, and we close the window
/// as soon as the frame is captured.
/// </summary>
public sealed class SigilWindow : GameWindow
{
    const int RenderSize = 2048; // supersampled; downsampled to OutputSize on save for anti-aliasing
    const int OutputSize = 1024;

    readonly List<Tri> _triangles;
    readonly string _outputPath;

    public SigilWindow(List<Tri> triangles, string outputPath)
        : base(
            GameWindowSettings.Default,
            new NativeWindowSettings
            {
                ClientSize = new Vector2i(64, 64),
                StartVisible = false,
                WindowBorder = WindowBorder.Hidden,
                Title = "sigil-offscreen",
                APIVersion = new Version(3, 3),
                Profile = ContextProfile.Core,
            })
    {
        _triangles = triangles;
        _outputPath = outputPath;
    }

    protected override void OnLoad()
    {
        base.OnLoad();
        RenderAndSave();
        Close();
    }

    void RenderAndSave()
    {
        int program = BuildShaderProgram();

        // GL.Disable(EnableCap.CullFace); 

        // --- Framebuffer: color texture + depth renderbuffer, rendered at supersample size ---
        int fbo = GL.GenFramebuffer();
        GL.BindFramebuffer(FramebufferTarget.Framebuffer, fbo);

        int colorTex = GL.GenTexture();
        GL.BindTexture(TextureTarget.Texture2D, colorTex);
        GL.TexImage2D(TextureTarget.Texture2D, 0, PixelInternalFormat.Rgba8, RenderSize, RenderSize, 0,
            PixelFormat.Rgba, PixelType.UnsignedByte, IntPtr.Zero);
        GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Linear);
        GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Linear);
        GL.FramebufferTexture2D(FramebufferTarget.Framebuffer, FramebufferAttachment.ColorAttachment0,
            TextureTarget.Texture2D, colorTex, 0);

        int depthRbo = GL.GenRenderbuffer();
        GL.BindRenderbuffer(RenderbufferTarget.Renderbuffer, depthRbo);
        GL.RenderbufferStorage(RenderbufferTarget.Renderbuffer, RenderbufferStorage.DepthComponent24, RenderSize, RenderSize);
        GL.FramebufferRenderbuffer(FramebufferTarget.Framebuffer, FramebufferAttachment.DepthAttachment,
            RenderbufferTarget.Renderbuffer, depthRbo);

        if (GL.CheckFramebufferStatus(FramebufferTarget.Framebuffer) != FramebufferErrorCode.FramebufferComplete)
            throw new InvalidOperationException("Offscreen framebuffer is incomplete.");

        GL.Viewport(0, 0, RenderSize, RenderSize);
        GL.Enable(EnableCap.DepthTest);
        GL.Enable(EnableCap.Blend);
        GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);

        // Parchment-ish background.
        GL.ClearColor(0.965f, 0.945f, 0.89f, 1f);
        GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

        // --- Upload sigil geometry (position-only triangles, normalized [0,1] xy + small z) ---
        int vao = GL.GenVertexArray();
        int vbo = GL.GenBuffer();
        GL.BindVertexArray(vao);
        GL.BindBuffer(BufferTarget.ArrayBuffer, vbo);

        float[] data = Tri.Triangles2Arr( _triangles );

        GL.BufferData(BufferTarget.ArrayBuffer, data.Length * sizeof(float), data, BufferUsageHint.StaticDraw);
        GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 3 * sizeof(float), 0);
        GL.EnableVertexAttribArray(0);

        GL.UseProgram(program);
        var proj = Matrix4.CreateOrthographicOffCenter(-1f, 1f, -1f, 1f, -0.1f, 0.1f);
        GL.UniformMatrix4(GL.GetUniformLocation(program, "uProjection"), false, ref proj);
        GL.Uniform4(GL.GetUniformLocation(program, "uColor"), new Vector4(0.09f, 0.08f, 0.10f, 1f));

        GL.DrawArrays(PrimitiveType.Triangles, 0, _triangles.Count);
        GL.Flush();

        // --- Read back and save ---
        byte[] pixels = new byte[RenderSize * RenderSize * 4];
        GL.ReadPixels(0, 0, RenderSize, RenderSize, PixelFormat.Rgba, PixelType.UnsignedByte, pixels);

        using var image = Image.LoadPixelData<Rgba32>(pixels, RenderSize, RenderSize);
        image.Mutate(ctx => ctx.Flip(FlipMode.Vertical)); // GL origin is bottom-left
        image.Mutate(ctx => ctx.Resize(new ResizeOptions
        {
            Size = new Size(OutputSize, OutputSize),
            Sampler = KnownResamplers.Lanczos3, // supersample downscale acts as anti-aliasing
        }));
        image.Save(_outputPath, new JpegEncoder { Quality = 92 });

        GL.DeleteVertexArray(vao);
        GL.DeleteBuffer(vbo);
        GL.DeleteFramebuffer(fbo);
        GL.DeleteTexture(colorTex);
        GL.DeleteRenderbuffer(depthRbo);
        GL.DeleteProgram(program);
    }

    static int BuildShaderProgram()
    {
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

        int vs = GL.CreateShader(ShaderType.VertexShader);
        GL.ShaderSource(vs, vertSrc);
        GL.CompileShader(vs);
        CheckShader(vs);

        int fs = GL.CreateShader(ShaderType.FragmentShader);
        GL.ShaderSource(fs, fragSrc);
        GL.CompileShader(fs);
        CheckShader(fs);

        int prog = GL.CreateProgram();
        GL.AttachShader(prog, vs);
        GL.AttachShader(prog, fs);
        GL.LinkProgram(prog);
        GL.GetProgram(prog, GetProgramParameterName.LinkStatus, out int linkStatus);
        if (linkStatus == 0)
            throw new InvalidOperationException("Shader link error: " + GL.GetProgramInfoLog(prog));

        GL.DeleteShader(vs);
        GL.DeleteShader(fs);
        return prog;
    }

    static void CheckShader(int shader)
    {
        GL.GetShader(shader, ShaderParameter.CompileStatus, out int status);
        if (status == 0)
            throw new InvalidOperationException("Shader compile error: " + GL.GetShaderInfoLog(shader));
    }
}

}