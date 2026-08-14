using pso;
using OpenTK.Mathematics;
using geo3d;

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
        Console.WriteLine( $"Eval: {dsgn}" );
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

        return soln;
    }


    /// <summary>
    /// Round up a quantity in [m] to the next 0.050
    /// </summary>
    public static float RoundUpToNext5cm( float meters ){
        int units = (int) (meters / 0.050f);
        if( (meters - units * 0.050f) > 0.0001f ){ return (units + 1) * 0.050f;  }
        return units * 0.050f;
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
        rtnLst.Capacity = Nradial * Ncircum;
        diameter_m = RoundUpToNext5cm( soln["D"] );
        zDepth_m   = RoundUpToNext5cm( soln["z"] );
        lFocus_m   = FocalLength_m( diameter_m, zDepth_m );
        Console.WriteLine( $"Design discretized paraboloid reflector with diameter {diameter_m:F2} [m], depth {zDepth_m:F2} [m], focal length {lFocus_m:F4} [m]," );
        Console.WriteLine( $"{Nradial} radial segments, and {Ncircum} circumferential segments" );

        // Console.WriteLine( $"f({0f}) = {QuadraticPositiveX( diameter_m, zDepth_m, 0f )}" );
        // Console.WriteLine( $"f({diameter_m/2}) = {QuadraticPositiveX( diameter_m, zDepth_m, diameter_m/2 )}" );
        float arcStep = 2f * MathF.PI / Ncircum;
        float radStep = diameter_m / 2f / (Nradial+1); // Central step is a flat polygon?
        float rad1, rad2;
        Vector3 v0, v1, v2, v3, mid1, mid2;
        Quad qd;
        for( int j = 0; j < Ncircum; ++j ){
            for( int i = 1; i <= Nradial; ++i ){
                rad1 = i*radStep;
                rad2 = (i+1)*radStep;
                v0   = new Vector3( rad1, 0f, QuadraticPositiveX( diameter_m, zDepth_m, rad1 ) );
                v1   = new Vector3( rad2, 0f, QuadraticPositiveX( diameter_m, zDepth_m, rad2 ) );
                v2   = new Vector3( rad2*MathF.Cos( arcStep ), rad2*MathF.Sin( arcStep ), QuadraticPositiveX( diameter_m, zDepth_m, rad2 ) );
                v3   = new Vector3( rad1*MathF.Cos( arcStep ), rad1*MathF.Sin( arcStep ), QuadraticPositiveX( diameter_m, zDepth_m, rad1 ) );
                qd   = new Quad( v0, v1, v2, v3 );
                mid1 = (v0+v3)/2f;
                mid2 = (v1+v2)/2f;
                qd.attrs["trapHeight"] = (mid2 - mid1).Length;
                qd.attrs["trapTop"]    = (v2 - v1).Length;
                qd.attrs["trapBottom"] = (v3 - v0).Length;
                Console.WriteLine( $"Trapezoid - Height: {qd.attrs["trapHeight"]}, Top: {qd.attrs["trapTop"]}, Bottom: {qd.attrs["trapBottom"]}, " );
                rtnLst.Add( qd );
            }
        }
        return rtnLst;
    }


    /// <summary>
    /// Design rib(s) as polygon(s)
    /// </summary>
    public static List<Polygon> DesignReflectorSupports( float matlThickness, List<Quad> segments, int Nradial = 5, int Ncircum = 20 ){
        List<Polygon> supports = [];
        List<Vector3> tempTop  = [];
        List<Vector3> tempBtm  = [];
        List<Vector3> top /**/ = [];
        List<Vector3> btm /**/ = [];
        supports.Capacity = Nradial*2;
        tempTop.Capacity  = Nradial*2;
        tempBtm.Capacity  = Nradial*2;        
        top.Capacity /**/ = Nradial+1;
        btm.Capacity /**/ = Nradial+1;
        Vector3 mid1, mid2, norm, p1, p2, p3, p4, vtx;
        float height = 0f;

        for( int i = 0; i < Nradial; ++i ){  height += segments[i].attrs["trapTop"];  }
        height /= Nradial;

        /// For one radial strip, Design one Radial Rib ///
        for( int i = 0; i < Nradial; ++i ){
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

        int j;
        top.Add( tempTop[0] );
        btm.Add( tempBtm[0] );        
        for( int i = 0; i < (Nradial-1); ++i ){
            j   = 2*i;
        
            p1  = tempTop[j  ];
            p2  = tempTop[j+1];
            p3  = tempTop[j+2];
            p4  = tempTop[j+4];
            vtx = MathVec3.ClosestPointBetweenLineSegments( p1, p2, p3, p4 );
            top.Add( vtx );

            p1  = tempBtm[j  ];
            p2  = tempBtm[j+1];
            p3  = tempBtm[j+2];
            p4  = tempBtm[j+4];
            vtx = MathVec3.ClosestPointBetweenLineSegments( p1, p2, p3, p4 );
            btm.Add( vtx );
        }
        top.Add( tempTop[^1] );
        btm.Add( tempBtm[^1] );        


        // FIXME: CONSTRUCT RIB(S)


        return supports;
    }

}

}