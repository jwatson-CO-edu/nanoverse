using OpenTK.Mathematics;
using Svg;

using pso;
using geo3d;
using geo2d;

namespace paraboloid {



/// <summary>
/// Useful constants for audible sound frequencies
/// </summary>
public class Constants {
    public const float _SPEED_OF_SOUND_MPS =  343f;
    public const float _BIRD_FREQ_LO /*-*/ = 3750f; // Brown Creeper, Source: https://www.allaboutbirds.org/news/do-bird-songs-have-frequencies-higher-than-humans-can-hear/
    public const float _BIRD_FREQ_HI /*-*/ = 1E4f; //- Blackpoll Warbler, Source: https://www.allaboutbirds.org/news/do-bird-songs-have-frequencies-higher-than-humans-can-hear/
}



/// <summary>
/// Cheap paraboloid dish designer for audible frequencies,
/// Source: https://diymics.com/parabolic-microphones/
/// </summary>
public class DishCalculator ( float lowestFreq_Hz, float Gdesired, float BWdesired_rad, 
                              float diaMax_m, float depthRatioMax, float focalLengthMax_m ) {

    /// Guidelines ///
    public const float _BAD_PARABOLOID_EFF = 0.5f;
    public const float _K_FACTOR_70DEG_RAD = 1.2217f;

    /// Rewards ///
    public const float _GAIN_REWARD /**/ = 20f; // Dimensionless 
    public const float _BEAMWIDTH_REWARD = 10f; // Radians

    /// Penalties ///
    public const float _DIAMETER_PENALTY = 10f; // Meters
    public const float _DEPTH_PENALTY    = 10f; // Meters
    public const float _FOCL_LEN_PENALTY = 10f; // Meters
    
    /// Constraints ///
    public float Gdesired /*---*/ = Gdesired;
    public float BWdesired_rad    = BWdesired_rad;
    public float diaMax_m /*---*/ = diaMax_m;
    public float depthRatioMax    = depthRatioMax;
    public float focalLengthMax_m = focalLengthMax_m;
    public float lowestFreq_Hz    = lowestFreq_Hz; // Lowest frequency the dish should respond to
    public float lambda_m /*---*/ = SoundFreq2Lambda_m( lowestFreq_Hz );
    
    /// Parameters ///
    public float diameter_m = 0f; // Dish diameter [m]
    public float zDepth_m   = 0f; // Vertical distance of dish edge from dish bottom [m]
    public float lFocus_m   = 0f; // Vertical distance of focus from dish bottom [m]
    public float a /*----*/ = 0f; // X divisor param [m]
    public float b /*----*/ = 0f; // Y divisor param [m]
    public int   radSegN    = 0; //- Number of radial segments
    public int   arcSegN    = 0; //- Number of circumferential segments

    /// Components ///
    public List<Vector3> backCirc  = [];
    public List<Tri>     backPlate = [];
    // public List<Tri>     cntrPlate = [];
    // public List<Tri>     edgePlate = [];


    /// <summary>
    /// Get wavelength [m] from frequency, ASSUMPTION: STP conditions ?,
    /// Source: https://en.wikipedia.org/wiki/Speed_of_sound
    /// </summary>
    public static float SoundFreq2Lambda_m( float freqHz ){
        return Constants._SPEED_OF_SOUND_MPS / freqHz;
    }


    /// <summary>
    /// Directivity Gain,
    /// Source: https://diymics.com/parabolic-microphones/
    /// </summary>
    public static float Gain( float dia_m, float lambda_m, float appEff = _BAD_PARABOLOID_EFF, bool asDecibel = false ){
        float rtnPwr = appEff * ((float) Math.Pow( (float) Math.PI * dia_m / lambda_m, 2 ));
        if( asDecibel ){  return 20f * (float) Math.Log10( rtnPwr );  }
        return rtnPwr;
    }


    /// <summary>
    /// Angular separation between the points on the antenna radiation pattern at which the power drops to one-half,
    /// Source: https://diymics.com/parabolic-microphones/
    /// </summary>
    public static float Beamwidth_rad( float dia_m, float lambda_m, float k = _K_FACTOR_70DEG_RAD ){
        return k * lambda_m / dia_m;
    }


    /// <summary>
    /// Get the distance from the "bottom" of the dish where the microphone should be placed,
    /// Sources: https://en.wikipedia.org/wiki/Parabola#As_a_graph_of_a_function, https://diymics.com/parabolic-microphones/
    /// </summary>
    public static float FocalLength_m( float dia_m, float dishDepth_m ){
        float rad_m = dia_m / 2f;
        // return MathF.Pow( dia_m / (16f * dishDepth_m), 2f );
        return MathF.Pow( rad_m, 2f ) / (4f * dishDepth_m);
    }


    /// <summary>
    /// Get the minimum diameter from the lowest expected frequency to be recorded + Desired Gain,
    /// Source: https://diymics.com/parabolic-microphones/
    /// </summary>
    public static float GetDiaFromFreqGain( float lowestFreq_Hz, float Gdesired, float appEff = _BAD_PARABOLOID_EFF ){
        float lambda_m = SoundFreq2Lambda_m( lowestFreq_Hz );
        return MathF.Sqrt( Gdesired / appEff ) * lambda_m / (float) Math.PI;
    }


    /// <summary>
    /// Get the minimum diameter from the lowest expected frequency to be recorded + Desired Beamwidth,
    /// Source: https://diymics.com/parabolic-microphones/
    /// </summary>
    public static float GetDiaFromFreqBW( float lowestFreq_Hz, float BWdesired_rad, float k = _K_FACTOR_70DEG_RAD ){
        float lambda_m = SoundFreq2Lambda_m( lowestFreq_Hz );
        return k * lambda_m / BWdesired_rad;
    }


    /// <summary>
    /// Scoring function for the PSO designer, See "PSO.cs"
    /// </summary>
    public float Score( DctVecF dsgn ){
        
        float f = FocalLength_m( dsgn["D"], dsgn["z"] );
        return Math.Min(Gain( dsgn["D"], lambda_m ), Gdesired) / Gdesired  * _GAIN_REWARD + // ---------------- Reward gain beyond desired / Penalize low
               (Beamwidth_rad( dsgn["D"], lambda_m ) - BWdesired_rad) / BWdesired_rad * _BEAMWIDTH_REWARD +  // Reward beamwidth beyond desired / Penalize low
               (diaMax_m - dsgn["D"]) / diaMax_m * _DIAMETER_PENALTY + // ------------------------------------- Penalize diameter beyond max / Reward small
               (depthRatioMax - dsgn["z"]/dsgn["D"]) / depthRatioMax * _DEPTH_PENALTY + // -------------------- Penalize relative depth beyond max / Reward shallow
               (focalLengthMax_m - f) / focalLengthMax_m * _FOCL_LEN_PENALTY; // ------------------------------ Penalize focal length beyond max / Reward short
    }


    /// <summary>
    /// Iteratively design the reflector by balancing practical considerations (Too lazy for closed form!),
    /// Sources: https://en.wikipedia.org/wiki/Particle_swarm_optimization#Algorithm
    /// </summary>
    public DctVecF DesignParabolicReflector(){
        
        PSOptimizer problem = new();
        problem.AddField( "D", 0.25f, 2.0f , 20 ); // Max diameter [m]
        problem.AddField( "z", 0.10f, 0.75f, 20 ); // Max depth [m]
        problem.PopulateInit( 300 );
        problem.SetScoringFunc( Score );
        DctVecF soln = problem.Solve( N : 12500 );
    
        Console.WriteLine( $"\n\nFrequency: {lowestFreq_Hz} [Hz]" );
        Console.WriteLine( $"Wavelength: {lambda_m:F4} [m]" );
        Console.WriteLine( $"Winning Solution:\n{soln}" );
        Console.WriteLine( $"Gain: ____ {Gain( soln["D"], lambda_m )}" );
        Console.WriteLine( $"Beamwidth: {Beamwidth_rad( soln["D"], lambda_m )/MathF.PI*180f} [deg]" );
        Console.WriteLine( $"Gain: ____ {Gain( 1f, lambda_m )}, (1m)" );
        Console.WriteLine( $"Beamwidth: {Beamwidth_rad( 1f, lambda_m )/MathF.PI*180f} [deg], (1m)\n" );

        diameter_m = soln["D"]; // --------------------------- Dish diameter [m]
        zDepth_m   = soln["z"]; // --------------------------- Vertical distance of dish edge from dish bottom [m]
        lFocus_m   = FocalLength_m( soln["D"], soln["z"] ); // Vertical distance of focus from dish bottom [m]

        return soln;
    }


    /// <summary>
    /// Round up a quantity in [m] to the next 0.050
    /// </summary>
    public static float RoundUpToNext5cm( float meters ){
        float unit_m = 0.050f;
        int units = (int) (meters / unit_m);
        if( (meters - units * unit_m) > 0.0001f ){ return (units + 1) * unit_m;  }
        return units * unit_m;
    }


    /// <summary>
    /// Round up a quantity in [m] to the next 0.050
    /// </summary>
    public static float RoundUpToNextUnit( float meters, float unit_m = 0.050f ){
        int units = (int) (meters / unit_m);
        if( (meters - units * unit_m) > 0.0001f ){ return (units + 1) * unit_m;  }
        return units * unit_m;
    }


    /// <summary>
    /// Quadratic Equation in Vertex Form, Positive X only
    /// </summary>
    public static float QuadraticPositiveX( float dia_m, float depth_m, float x_m ){
        float rad_m = dia_m / 2f;
        return depth_m * MathF.Pow( x_m, 2f ) / MathF.Pow( rad_m, 2f ) - depth_m;
    }


    /// <summary>
    /// Output `Quad`s representing one radial "slice" of the discretized dish, Not including the central poly
    /// </summary>
    public List<Quad> SegmentDesignedReflector( DctVecF soln, int Nradial = 5, int Ncircum = 20 ){
        List<Quad> rtnLst = [];
        List<Quad> petal;
        float /**/ designUnit_m = 0.010f;
        radSegN = Nradial; //- Number of radial segments
        arcSegN = Ncircum; // 
        rtnLst.Capacity = Nradial * Ncircum * 2;
        backCirc.Capacity = Ncircum;
        backPlate.Capacity = Ncircum;

        // diameter_m = RoundUpToNext5cm( soln["D"] );
        // zDepth_m   = RoundUpToNext5cm( soln["z"] );
        
        diameter_m = RoundUpToNextUnit( soln["D"], unit_m : designUnit_m );
        zDepth_m   = RoundUpToNextUnit( soln["z"], unit_m : designUnit_m );
        
        lFocus_m   = FocalLength_m( diameter_m, zDepth_m );
        Console.WriteLine( $"Design discretized paraboloid reflector with diameter {diameter_m:F2} [m], depth {zDepth_m:F2} [m], focal length {lFocus_m:F4} [m]," );
        Console.WriteLine( $"{Nradial} radial segments, and {Ncircum} circumferential segments" );

        float arcStep = 2f * MathF.PI / Ncircum;
        float radStep = diameter_m / 2f / (Nradial+1); // Central step is a flat polygon?
        float rad1, rad2;
        Vector3 v0, v1, v2, v3, mid1, mid2;
        Quad qd;//, qr;
        for( int j = 0; j < Ncircum; ++j ){
            petal = [];
            for( int i = 1; i <= Nradial; ++i ){
                rad1 = i*radStep;
                rad2 = (i+1)*radStep;
                v0   = new Vector3( rad1, 0f, QuadraticPositiveX( diameter_m, zDepth_m, rad1 ) );
                v1   = new Vector3( rad2, 0f, QuadraticPositiveX( diameter_m, zDepth_m, rad2 ) );
                v2   = new Vector3( rad2*MathF.Cos( arcStep ), rad2*MathF.Sin( arcStep ), QuadraticPositiveX( diameter_m, zDepth_m, rad2 ) );
                v3   = new Vector3( rad1*MathF.Cos( arcStep ), rad1*MathF.Sin( arcStep ), QuadraticPositiveX( diameter_m, zDepth_m, rad1 ) );
                qd   = new Quad( v0, v1, v2, v3 );
                // qr   = new Quad( v3 + eps, v2 + eps, v1 + eps, v0 + eps );
                mid1 = (v0+v3)/2f;
                mid2 = (v1+v2)/2f;

                qd.attrs["trapHeight"] = (mid2 - mid1).Length;
                qd.attrs["trapTop"]    = (v2 - v1).Length;
                qd.attrs["trapBottom"] = (v3 - v0).Length;
                
                Console.WriteLine( $"Trapezoid - Height: {qd.attrs["trapHeight"]}, Top: {qd.attrs["trapTop"]}, Bottom: {qd.attrs["trapBottom"]}, " );
                petal.Add( qd );
                // petal.Add( qr );
            }
            petal = Quad.RotateMesh( petal, Vector3.UnitZ, j*arcStep );
            backCirc.Add( petal[0].V0() );
            rtnLst.AddRange( petal );
        }

        Vector3 curPnt;
        Vector3 lstPnt = backCirc[^1];
        Vector3 center = MathVec3.UniformPointCentroid( backCirc );
        for( int j = 0; j < Ncircum; ++j ){
            curPnt = backCirc[j];
            backPlate.Add( new Tri( center, curPnt, lstPnt ) );
            lstPnt = curPnt;
        }

        return rtnLst;
    }


    /// <summary>
    /// Design rib(s) as polygon(s)
    /// </summary>
    public List<Quad> DesignReflectorSupports( float matlThickness, List<Quad> segments ){
        List<Quad>    support  = [];
        List<Quad>    supports = [];
        List<Vector3> tempTop  = [];
        List<Vector3> tempBtm  = [];
        supports.Capacity = radSegN*arcSegN*2;
        tempTop.Capacity  = radSegN*2;
        tempBtm.Capacity  = radSegN*2;        
        Vector3 mid1, mid2, norm, p1, p2, p3, p4;
        float height = 0f;
        float dTheta = MathF.PI * 2 / arcSegN;

        for( int i = 0; i < radSegN; ++i ){  height += segments[i].attrs["trapHeight"];  }
        height /= radSegN;
        height *= 0.75f;

        /// For one radial strip, Design one Radial Rib ///
        for( int i = 0; i < radSegN; ++i ){

            mid1 = (segments[i].V0() + segments[i].V3())/2f;
            mid2 = (segments[i].V1() + segments[i].V2())/2f;
            norm = Vector3.Cross( segments[i].V2() - segments[i].V1(), segments[i].V0() - segments[i].V1() ).Normalized(); 
            
            mid1 -= norm * matlThickness;
            mid2 -= norm * matlThickness;
            tempTop.Add( mid1 );
            tempTop.Add( mid2 );

            mid1 -= norm * height;
            mid2 -= norm * height;
            tempBtm.Add( mid1 );
            tempBtm.Add( mid2 );
        }

        for( int i = 0; i < (tempTop.Count-1); ++i ){
            p1  = tempTop[i  ];
            p2  = tempBtm[i  ];
            p3  = tempBtm[i+1];
            p4  = tempTop[i+1];
            support.Add( new Quad( p1, p2, p3, p4 ) );
        }

        Console.WriteLine( $"There are ${support.Count} `Quad`s in the designed support." );

        for( int i = 0; i < arcSegN; ++i ){
            supports.AddRange( Quad.RotateMesh( support, Vector3.UnitZ, i*dTheta ) );
        }

        return supports;
    }


    /// <summary>
    /// Get one "petal" from the design `Quad`s as a collection of line `Segment`s
    /// </summary>
    public List<Segment> PetalSegments( List<Quad> reflectorQuads ){
        List<Segment> fig   = [];
        Segment seg;
        List<Quad>    petal = reflectorQuads[0..radSegN];
        fig.Capacity = 4 + petal.Count - 1;
        float height = 0f;
        foreach( Quad q in petal ){  
            Console.WriteLine(q);
            height += q.attrs["trapHeight"];
        }
        float segHlf, y_i = -height/2f;
        float y_l = -height/2f;
        float topLen = petal[^1].attrs["trapTop"   ];
        float btmLen = petal[0 ].attrs["trapBottom"];
        float lstHlf = btmLen/2f;

        // fig.AddRange( Figure.MakeTrapezoid( height, topLen, btmLen ) );
        Vector2 upRght = new(  topLen/2f,  height/2f );
        Vector2 upLeft = new( -topLen/2f,  height/2f );
        Vector2 loLeft = new( -btmLen/2f, -height/2f );
        Vector2 loRght = new(  btmLen/2f, -height/2f );

        seg = new( upRght, upLeft );
        seg.SetColor( new Vector3(1,0,0) );
        fig.Add( seg );

        seg = new( loLeft, loRght );
        seg.SetColor( new Vector3(1,0,0) );
        fig.Add( seg );
        
        // for( int i = 1; i < petal.Count; ++i ){
        for( int i = 0; i < petal.Count; ++i ){

            y_i   += petal[i].attrs["trapHeight"];
            segHlf = petal[i].attrs["trapTop"] / 2f;

            if( i < petal.Count-1 ){
                seg = new Segment( new Vector2( -segHlf, y_i ), new Vector2( segHlf, y_i ) );
                seg.SetColor( new Vector3(0,0,1) );
                fig.Add( seg );
            }

            seg = new Segment( new Vector2( -lstHlf, y_l ), new Vector2( -segHlf, y_i ) );
            seg.SetColor( new Vector3(1,0,0) );
            fig.Add( seg );

            seg = new Segment( new Vector2( lstHlf, y_l ), new Vector2( segHlf, y_i ) );
            seg.SetColor( new Vector3(1,0,0) );
            fig.Add( seg );

            y_l = y_i;
            lstHlf = segHlf;
            
        }
        Segment.SetWeight( fig, 0f ); // Hairline
        return fig;
    }


    /// <summary>
    /// Get one "strut" from the support `Quad`s as a collection of line `Segment`s
    /// </summary>
    public List<Segment> SupportSegments( List<Quad> supportQuads ){
        Segment /*-*/ seg_i;
        List<Segment> fig = [];
        int /*-----*/ N   = 2 * radSegN - 1;
        Console.WriteLine( $"Fetch the first {N} `Quad`s" );
        List<Quad>    strut = supportQuads[0..N];
        List<LinSeg>  perim = Quad.GetPerimeter( strut );
        List<Vector3> pPnts = [];
        pPnts.Capacity = pPnts.Count * 2;
        foreach( LinSeg seg in perim ){
            pPnts.Add( seg.V0() );
            pPnts.Add( seg.V1() );
        }
        List<Vector2> fPnts = Ops2D3D.Project3dPointsTo2d( pPnts );
        N = perim.Count;
        int j;
        for( int i = 0; i < N; ++i ){
            j = 2*i;
            seg_i = new Segment( fPnts[j], fPnts[j+1] );
            seg_i.SetColor( new Vector3(1,0,0) );
            fig.Add( seg_i );
        }

        return fig;   
    }


    /// <summary>
    /// Layout a design ready for cutting
    /// </summary>
    public List<Segment> BackSegments(){
        List<Segment> fig   = [];
        List<Vector2> fPnts = Ops2D3D.Project3dPointsTo2d( backCirc );
        Vector2 /*-*/ lst   = fPnts[^1];
        Segment /*-*/ seg_i;
        foreach(Vector2 pnt in fPnts ){
            seg_i = new Segment( pnt, lst );
            fig.Add( seg_i );
            lst = pnt; 
        }
        return fig;
    }


}

}