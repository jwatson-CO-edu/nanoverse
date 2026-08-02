using curve;
using helpers;
using OpenTK.Mathematics;

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
    public Vector3 /**/ _Z_DIR       = new(0,0,1);



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
            int /*---------*/ Nintersect;
            int /*---------*/ interChoice;
            List<List<float>> intersections;

            
            // For each existing stroke
            for( int i = 0; i < count; ++i ){

                // Disre
                if( nuStroke.HasNeighbor( strokes[i] ) ){  continue;  }

                // Scan for intersections
                intersections = nuStroke.GetIntersections( strokes[i], 1f/128f, thick );
                Nintersect    = intersections[0].Count / 2;
                // For each intersection  
                for( int j = 0; j < Nintersect; ++j ){
                    interChoice = random.Next(3);
                    switch( interChoice ){
                        /// Cross ///
                        case 0:
                            // FIXME: REMOVE UNDERPAINT FROM THIS SPAN
                            break;
                        /// Above ///
                        case 1:
                            // FIXME: BUMP OFFSET UP
                            break;
                        /// Below ///
                        case 2:
                            // FIXME: BUMP OFFSET DOWN
                            break;
                        /// This should NOT happen! ///
                        default:
                            throw new InvalidDataException( $"{interChoice} was NOT a valid choice" );
                    }
                }
            }

            // CREATE STROKE
                // SCAN FOR INTERSECTIONS
                // FOR EACH INTERSECTION: SELECT ONE {CROSS, ABOVE, BELOW}
            // CREATE UNDERSTROKE
            
            if( random.NextSingle() < breakProb ){  break;  } // Roll for break
            count++;
            lastCurv = strokes[ random.Next( count ) ].curve;
        }

        // FIXME: FOR EACH STROKE
            // CREATE GEO
                // STROKE
                // UNDERSTROKE
    }

}

}