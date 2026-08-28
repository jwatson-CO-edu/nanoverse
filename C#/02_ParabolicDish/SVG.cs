using OpenTK.Mathematics;
using Svg;
using System.Drawing;
using geo2d;


////////////////////////////////////////////////////////////////////////////////////////////////////
////////// SCALABLE VECTOR GRAPHICS ////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


namespace cut_svg {


public class CutSVG {

    /// Members ///

    // Page / layout configuration //
    public const float _HAIRLINE_INCH = 0.001f;
    public const float _IN_TO_M = 0.0254f; // 1 inch = 0.0254 m, exactly
    public const float _M_TO_IN = 1f / 0.0254f; // 1 inch = 0.0254 m, exactly

    // Page / layout configuration //
    public float /*-*/ PageWidthIn;   // US Letter
    public float /*-*/ PageHeightIn;
    public float /*-*/ MarginIn;
    
    // Document Structur //
    public SvgDocument doc;
    public SvgGroup    cutGroup;
    public SvgGroup    scoreGroup;


    /// Default Constructor ///
    public CutSVG(){
        PageWidthIn  =  8.5f;   
        PageHeightIn = 11f;
        MarginIn     =  0.5f;
        doc /*----*/ = new(){
            Width   = new SvgUnit( SvgUnitType.Inch, PageWidthIn  ),
            Height  = new SvgUnit( SvgUnitType.Inch, PageHeightIn ),
            ViewBox = new SvgViewBox( 0, 0, PageWidthIn, PageHeightIn ),
        };
        cutGroup   = new SvgGroup { ID = "cut_outer_red"    };
        scoreGroup = new SvgGroup { ID = "score_inner_blue" };
        doc.Children.Add( cutGroup   );
        doc.Children.Add( scoreGroup );
    }


    static SvgUnit InchUnitFromMeters( float meters ) => new( SvgUnitType.User, meters * _M_TO_IN );


    public void AddCutSegment_m( Segment segment ){
        SvgLine line = new(){
            StartX /**/ = InchUnitFromMeters( segment.V0().X ),
            StartY /**/ = InchUnitFromMeters( segment.V0().Y ),
            EndX /*--*/ = InchUnitFromMeters( segment.V1().X ),
            EndY /*--*/ = InchUnitFromMeters( segment.V1().Y ),
            Stroke /**/ = new SvgColourServer( Color.FromArgb( 255, 255, 0, 0 ) ),
            StrokeWidth = new SvgUnit( SvgUnitType.User, _HAIRLINE_INCH ),
            Fill /*--*/ = SvgPaintServer.None, // NEVER Fill!
        };
        cutGroup.Children.Add( line );
    }


    public void AddScoreSegment_m( Segment segment ){
        SvgLine line = new(){
            StartX /**/ = InchUnitFromMeters( segment.V0().X ),
            StartY /**/ = InchUnitFromMeters( segment.V0().Y ),
            EndX /*--*/ = InchUnitFromMeters( segment.V1().X ),
            EndY /*--*/ = InchUnitFromMeters( segment.V1().Y ),
            Stroke /**/ = new SvgColourServer( Color.FromArgb( 255, 0, 0, 255 ) ),
            StrokeWidth = new SvgUnit( SvgUnitType.User, _HAIRLINE_INCH ),
            Fill /*--*/ = SvgPaintServer.None, // NEVER Fill!
        };
        scoreGroup.Children.Add( line );
    }


    public void AddSegments_m( IEnumerable<Segment> segments ){
        foreach( Segment seg in segments ){
            if( seg.C0()[2] > 0.75 ){  AddScoreSegment_m( seg );  }else{  AddCutSegment_m( seg );  }
        }
    }


    public void PatternGroup( IEnumerable<Segment> segments, Vector2 origin, Vector2 step_m, float dTheta_rad, int Nstep = 2 ){
        List<Segment> lstGrp  = Segment.ShiftSegments( segments, origin );
        List<Segment> lstRot  = Segment.RotateSegments( segments, new Vector2(), dTheta_rad );
        Vector2 /*-*/ totStep = origin + step_m;

        AddSegments_m( lstGrp );

        for( int i = 1; i < Nstep; ++i ){
            AddSegments_m( Segment.ShiftSegments( lstRot, totStep ) );
            totStep += step_m;
            lstRot   = Segment.RotateSegments( lstRot, new Vector2(), dTheta_rad );
        }
    }


    public void WriteSVG( string outputPath = "OUTPUT.svg" ){
        doc.Write( outputPath );
    }

}


}