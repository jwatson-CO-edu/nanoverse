namespace pso {

/// <summary>
/// Particle Swarm Optimization
/// </summary>
public class PSOptimizer {

    public List<string> /*--------------------*/ fields    = [];
    public Dictionary<string,float[]> /*------*/ ranges    = [];
    public LinkedList<Dictionary<string,float>>  particles = [];
    public Dictionary<string,float> /*--------*/ bestPrtcl = [];
    public Dictionary<string,float> /*--------*/ particle  = [];
    public Func<Dictionary<string,float>,float>? Score     = null;
    public Random /*--------------------------*/ rand /**/ = new();

    
    /// <summary>
    /// Set domain [lo,hi,div,step,] for one field
    /// </summary>
    public void AddField( string field, float lo, float hi, int div = 10 ){
        if( lo > hi ){  throw new Exception( $"Domain [{lo}, {hi}] is MALFORMED!" );  }
        ranges[ field ] = [lo, hi, div, (hi-lo)/div,];
    }


    /// <summary>
    /// Set function that will be used for scoring each particle
    /// </summary>
    public void SetScoringFunc( Func<Dictionary<string,float>,float>? func ){  Score = func;  }


    /// <summary>
    /// Generate a random particle at a grid point
    /// </summary>
    public Dictionary<string,float> RandParticle(){
        Dictionary<string,float> rtnPrt = [];
        foreach( string field in fields ){
            rtnPrt[ field ] = ranges[ field ][0] + rand.Next( (int) ranges[ field ][2] + 1 ) * ranges[ field ][3];
        }
        rtnPrt[ "score" ] = float.NaN;
        return rtnPrt;
    }


    /// <summary>
    /// Generate N random particles at grid points
    /// </summary>
    public void PopulateInit( int N = 0 ){
        if( N < 1 ){  
            N = 1;   
            foreach( string field in fields ){ N *= (int) ranges[ field ][2];  }
            N /= 2;
        }
        particles.Clear();
        for( int i = 0; i < N; ++i ){  particles.AddLast( RandParticle() );  }
    }


    public Dictionary<string,float> Solve( /* FIXME: Conditions - N or Low Velocity Factor */ ){
        
        // FIXME: START HERE, https://en.wikipedia.org/wiki/Particle_swarm_optimization#Algorithm
        
        return bestPrtcl;
    }


}

}