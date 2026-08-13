using pso;

namespace paraboloid {

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
    /// Get the distance from the "bottom" of the dish where the microphone should be placed ,
    /// Source: https://diymics.com/parabolic-microphones/
    /// </summary>
    public static float FocalLength_m( float dia_m, float dishDepth_m ){
        return (float) Math.Pow( dia_m / (16f * dishDepth_m), 2 );
    }


    /// <summary>
    /// Get the minimum diameter from the lowest expected frequency to be recorded + Desired Gain,
    /// Source: https://diymics.com/parabolic-microphones/
    /// </summary>
    public static float GetDiaFromFreqGain( float lowestFreq_Hz, float Gdesired, float appEff = _BAD_PARABOLOID_EFF ){
        float lambda_m = SoundFreq2Lambda_m( lowestFreq_Hz );
        return (float) Math.Sqrt( Gdesired / appEff ) * lambda_m / (float) Math.PI;
    }


    /// <summary>
    /// Get the minimum diameter from the lowest expected frequency to be recorded + Desired Beamwidth,
    /// Source: https://diymics.com/parabolic-microphones/
    /// </summary>
    public static float GetDiaFromFreqBW( float lowestFreq_Hz, float BWdesired_rad, float k = _K_FACTOR_70DEG_RAD ){
        float lambda_m = SoundFreq2Lambda_m( lowestFreq_Hz );
        return k * lambda_m / BWdesired_rad;
    }


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
    public void DesignParabolicReflector(){
        
        PSOptimizer problem = new();
        problem.AddField( "D", 0.25f, 2.0f  ); // Max diameter [m]
        problem.AddField( "z", 0.10f, 0.75f ); // Max depth [m]
        problem.PopulateInit();
        problem.SetScoringFunc( Score );
        DctVecF soln = problem.Solve( N : 100000 );
    
        Console.WriteLine( $"\n\nFrequency: {lowestFreq_Hz} [Hz]" );
        Console.WriteLine( $"Wavelength: {lambda_m:F4} [m]" );
        Console.WriteLine( $"Winning Solution:\n{soln}" );
        Console.WriteLine( $"Gain: ____ {Gain( soln["D"], lambda_m )  }" );
        Console.WriteLine( $"Beamwidth: {Beamwidth_rad( soln["D"], lambda_m )/MathF.PI*180f} [deg]" );
        Console.WriteLine( $"Gain: ____ {Gain( 1f, lambda_m )}, (1m)" );
        Console.WriteLine( $"Beamwidth: {Beamwidth_rad( 1f, lambda_m )/MathF.PI*180f} [deg], (1m)\n" );

    }


    /// <summary>
    /// Produce a cutting pattern for a discretized paraboloid dish,
    /// Sources: 
    /// </summary>
    public void SegmentDesignedReflector(){

    }



}

}