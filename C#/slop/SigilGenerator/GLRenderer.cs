using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.Formats.Jpeg;
using SixLabors.ImageSharp.PixelFormats;
using SixLabors.ImageSharp.Processing;

namespace SigilGenerator;

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

    readonly List<Vector3> _triangles;
    readonly string _outputPath;

    public SigilWindow(List<Vector3> triangles, string outputPath)
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

        float[] data = new float[_triangles.Count * 3];
        for (int i = 0; i < _triangles.Count; i++)
        {
            data[i * 3] = _triangles[i].X;
            data[i * 3 + 1] = _triangles[i].Y;
            data[i * 3 + 2] = _triangles[i].Z;
        }
        GL.BufferData(BufferTarget.ArrayBuffer, data.Length * sizeof(float), data, BufferUsageHint.StaticDraw);
        GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 3 * sizeof(float), 0);
        GL.EnableVertexAttribArray(0);

        GL.UseProgram(program);
        var proj = Matrix4.CreateOrthographicOffCenter(0f, 1f, 0f, 1f, -1f, 1f);
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
