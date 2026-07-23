# SigilGenerator

Procedurally generates a random sigil and rasterizes it to a unique JPG using a real
OpenGL 3.3 pipeline (via OpenTK 4), rendered offscreen at 2048x2048 and downsampled
to 1024x1024 for anti-aliasing, then encoded with SixLabors.ImageSharp.

## How it satisfies the brief

- **Node/edge graph model**: `SigilFactory.Generate` grows a handful of "pen strokes"
  from a shared pool of branch points (`branchNodes`), so later strokes can start from
  earlier strokes' waypoints or circle perimeters — that's the branching.
- **Varying line thickness**: each `StrokePoint` carries its own width; ribbons are built
  by lerping width along every segment, and stroke tips are randomly tapered to a point.
- **Square boundary**: all generation happens in normalized `[0,1]x[0,1]` space with a
  13% margin, clamped every step, then mapped 1:1 to the square render target.
- **Curves used sparingly, small circles**: each stroke segment is either a sudden sharp
  turn (~50% chance) or a gentle multi-step curve; small circles (radius ~0.018-0.045 of
  the canvas) are only occasionally attached, and their perimeter points feed back into
  the branch-node pool so later strokes can spring off them.
- **Sudden angles or gentle curves**: implemented directly in the two turn modes above.
- **Crossings with visible over/under breaks**: after all strokes are generated,
  `SigilFactory.ComputeCrossingGaps` finds every segment-segment intersection between
  *different* strokes (ignoring near-endpoint intersections, which are treated as
  deliberate joins/nodes, not crossings) and cuts a small gap into whichever stroke has
  the lower `Layer` (drawn earlier = passes underneath). `MeshBuilder` rounds off the cut
  edges so breaks read as intentional rather than jagged.
- **Small Z variation for pseudo-3D layering**: each stroke gets a tiny `Z` offset based
  on draw order; depth testing is enabled in the renderer.
- **1024x1024 JPG, unique filename**: `sigil_{seed}_{guid}.jpg`, written next to the
  executable.

## Project layout

- `MathUtil.cs` — 2D vector helpers, segment/segment intersection.
- `SigilModel.cs` — `Stroke`/`StrokePoint` data model + the procedural generator.
- `MeshBuilder.cs` — turns strokes into ribbon-quad + round-cap/joint triangles, honoring
  the computed gaps.
- `GLRenderer.cs` — a hidden `GameWindow` that owns the GL context just long enough to
  render one offscreen frame (FBO + basic flat-color shader) and save it.
- `Program.cs` — entry point: generate → build mesh → render → save.

## Build & run

Requires the .NET 8 SDK.

```bash
cd SigilGenerator
dotnet restore
dotnet run                # random seed
dotnet run -- 12345       # fixed seed, reproducible sigil
```

Each run produces one `sigil_<seed>_<guid>.jpg` (1024x1024) in the working directory.

### Headless Linux note

OpenTK creates its GL context through GLFW, which needs a windowing backend even for an
invisible window. On a machine with no X11/Wayland session (e.g. a bare CI container),
run it under a virtual framebuffer:

```bash
xvfb-run -a dotnet run
```

On a normal desktop (Windows, macOS, or Linux with a display), it runs as-is — the
window is created hidden (`StartVisible = false`, 64x64) and is only a vehicle for the
GL context; nothing is shown on screen.

## Tuning

Everything visually interesting is driven by a handful of constants/ranges in
`SigilFactory.Generate` (stroke count, segment count, turn probability/angle range,
thickness range, circle probability/radius) and `GLRenderer` (canvas colors, supersample
resolution). Nothing else needs to change to get a different "family" of sigils.
