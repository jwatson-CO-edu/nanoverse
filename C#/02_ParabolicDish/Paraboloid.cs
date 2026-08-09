namespace paraboloid {

/// <summary>
/// Cheap paraboloid dish designer for audible frequencies
/// </summary>
public class DishCalculator {

    public float lowestFreq = 0f; // Lowest frequency the dish should respond to
    public float diameter_m = 0f; // Dish diameter [m]
    public float dFocus_m   = 0f; // Depth of focus from dish edge [m]
    public float a /*----*/ = 0f; // X divisor param
    public float b /*----*/ = 0f; // Y divisor param
    public int   radSegN    = 0; //- Number of radial segments
    public int   arcSegN    = 0; //- Number of circumferential segments


    public static float Gain( float dia_m, float lambda_m, float appEff = 0.55f ){
        return (float) Math.Pow( (float) Math.PI * dia_m / lambda_m, 2 );
    }

    /// <summary>
    /// Angular separation between the points on the antenna radiation pattern at which the power drops to one-half 
    /// Source: 
    /// </summary>
    public static float Beamwidth_rad( float dia_m, float lambda_m, float k = 1.221f ){
        return k * lambda_m / dia_m;
    }


    /// <summary>
    /// Get the minimum diameter from the lowest expected frequency to be recorded 
    /// Source: 
    /// </summary>
    public float GetDiaFromFreq_m( float freq_Hz ){
        return freq_Hz+a+b;
    }



    /// <summary>
    /// Iteratively design the antenna by balancing practical considerations 
    /// Sources: 
    /// </summary>
    public void DesignParabolicAntenna(){

    }


    /// <summary>
    /// Produce a cutting pattern for a discretized paraboloid dish
    /// Sources: 
    /// </summary>
    public void SegmentDesignedAntenna(){

    }



}

}