namespace paraboloid {

public class Constants {
    public const float _SPEED_OF_SOUND_MPS =  343f;
    public const float _BAD_PARABOLOID_EFF =    0.5f;
    public const float _K_FACTOR_70DEG_RAD =    1.2217f;
    public const float _BIRD_FREQ_LO /*-*/ = 3750f; // Brown Creeper, Source: https://www.allaboutbirds.org/news/do-bird-songs-have-frequencies-higher-than-humans-can-hear/
    public const float _BIRD_FREQ_HI /*-*/ = 1E4f; //- Blackpoll Warbler, Source: https://www.allaboutbirds.org/news/do-bird-songs-have-frequencies-higher-than-humans-can-hear/
}

/// <summary>
/// Cheap paraboloid dish designer for audible frequencies
/// </summary>
public class DishCalculator {

    public float lowestFreq = 0f; // Lowest frequency the dish should respond to
    public float diameter_m = 0f; // Dish diameter [m]
    public float lFocus_m   = 0f; // Deistance of focus from dish bottom [m]
    public float a /*----*/ = 0f; // X divisor param
    public float b /*----*/ = 0f; // Y divisor param
    public int   radSegN    = 0; //- Number of radial segments
    public int   arcSegN    = 0; //- Number of circumferential segments


    /// <summary>
    /// Get wavelength [m] from frequency, ASSUMPTION: STP conditions ?
    /// Source: https://en.wikipedia.org/wiki/Speed_of_sound
    /// </summary>
    public static float SoundFreq2Lambda_m( float freqHz ){
        return Constants._SPEED_OF_SOUND_MPS / freqHz;
    }


    /// <summary>
    /// Directivity Gain 
    /// Source: https://diymics.com/parabolic-microphones/
    /// </summary>
    public static float Gain( float dia_m, float lambda_m, float appEff = Constants._BAD_PARABOLOID_EFF, bool asDecibel = false ){
        float rtnPwr = appEff * ((float) Math.Pow( (float) Math.PI * dia_m / lambda_m, 2 ));
        if( asDecibel ){  return 20f * (float) Math.Log10( rtnPwr );  }
        return rtnPwr;
    }


    /// <summary>
    /// Angular separation between the points on the antenna radiation pattern at which the power drops to one-half 
    /// Source: https://diymics.com/parabolic-microphones/
    /// </summary>
    public static float Beamwidth_rad( float dia_m, float lambda_m, float k = Constants._K_FACTOR_70DEG_RAD ){
        return k * lambda_m / dia_m;
    }


    /// <summary>
    /// Get the distance from the "bottom" of the dish where the microphone should be placed 
    /// Source: https://diymics.com/parabolic-microphones/
    /// </summary>
    public static float FocalLength_m( float dia_m, float dishDepth_m ){
        return (float) Math.Pow( dia_m / (16f * dishDepth_m), 2 );
    }


    /// <summary>
    /// Get the minimum diameter from the lowest expected frequency to be recorded + Desired Gain
    /// Source: https://diymics.com/parabolic-microphones/
    /// </summary>
    public static float GetDiaFromFreqGain( float lowestFreq_Hz, float Gdesired, float appEff = Constants._BAD_PARABOLOID_EFF ){
        float lambda_m = SoundFreq2Lambda_m( lowestFreq_Hz );
        return (float) Math.Sqrt( Gdesired / appEff ) * lambda_m / (float) Math.PI;
    }


    /// <summary>
    /// Get the minimum diameter from the lowest expected frequency to be recorded + Desired Beamwidth
    /// Source: https://diymics.com/parabolic-microphones/
    /// </summary>
    public static float GetDiaFromFreqBW( float lowestFreq_Hz, float BWdesired_rad, float k = Constants._K_FACTOR_70DEG_RAD ){
        float lambda_m = SoundFreq2Lambda_m( lowestFreq_Hz );
        return k * lambda_m / BWdesired_rad;
    }


    /// <summary>
    /// Represents a single point in the design space of Parabolic Reflectors
    /// Source: https://diymics.com/parabolic-microphones/
    /// </summary>
    class DishDesign ( float lowestFreq_Hz, float D, float z ) {

        /// Rewards ///
        public const float _GAIN_REWARD /**/ = 10f;
        public const float _BEAMWIDTH_REWARD = 10f;

        /// Penalties ///
        public const float _DIAMETER_PENALTY = 10f;
        public const float _DEPTH_PENALTY    =  5f;
        public const float _FOCL_LEN_PENALTY = 10f;

        /// Parameters ///
        public float freqHz = lowestFreq_Hz;
        public float lambda = SoundFreq2Lambda_m( lowestFreq_Hz );
        public float D = D;
        public float z = z;
        public float f = FocalLength_m( D, z );

        public float Score( float diaMax_m, float depthRatioMax, float focalLengthMax_m ){
            return Gain( D, lambda )  * _GAIN_REWARD +
                   Beamwidth_rad( D, lambda ) * _BEAMWIDTH_REWARD +
                   (diaMax_m - D) * _DIAMETER_PENALTY +
                   (depthRatioMax - z/D) * _DEPTH_PENALTY +
                   (focalLengthMax_m - f) * _FOCL_LEN_PENALTY;
        }

    }


    /// <summary>
    /// Iteratively design the reflector by balancing practical considerations (Too lazy for closed form!)
    /// Sources: https://diymics.com/parabolic-microphones/
    /// </summary>
    public void DesignParabolicReflector( float lowestFreq_Hz, float Gdesired, float BWdesired_rad, 
                                          float diaMax_m, float depthRatioMax, float focalLengthMax_m,  
                                          float deltaHalt = 1f ){
        float lastScore  = 0f;
        
        Queue<DishDesign> designs = [];
        List<DishDesign>  dsgnLst = [];

        float /**/ D_gn  = GetDiaFromFreqGain( lowestFreq_Hz, Gdesired );
        float /**/ D_bw  = GetDiaFromFreqBW( lowestFreq_Hz, BWdesired_rad );
        float /**/ D     = (D_gn + D_bw) / 2f;
        float /**/ zDpth = D * depthRatioMax;
        DishDesign dsgn  = new( lowestFreq, D, zDpth );
        float /**/ score = dsgn.Score( diaMax_m, depthRatioMax, focalLengthMax_m );
        float /**/ delta = score - lastScore;

        designs.Enqueue( dsgn );

        while( delta > deltaHalt ){

        }

    }


    /// <summary>
    /// Produce a cutting pattern for a discretized paraboloid dish
    /// Sources: 
    /// </summary>
    public void SegmentDesignedReflector(){

    }



}

}