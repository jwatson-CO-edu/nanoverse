namespace pso {

/// <summary>
/// Particle Swarm Optimization
/// </summary>
public class PSOptimizer {

    public class Particle {
        public Dictionary<string,float> position = [];
        public Dictionary<string,float> velocity = [];
        public Dictionary<string,float> bestPosn = [];
    }

    public List<string> /*--------------------*/ fields    = [];
    public Dictionary<string,float[]> /*------*/ ranges    = [];
    public LinkedList<Particle> /*------------*/ particles = [];
    public Particle /*------------------------*/ bestPrtcl = new();
    public Particle /*------------------------*/ particle  = new();
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
    public Particle RandParticle(){
        Particle rtnPrt = new();
        float    value;
        foreach( string field in fields ){
            value = ranges[ field ][0] + rand.Next( (int) ranges[ field ][2] + 1 ) * ranges[ field ][3];
            rtnPrt.position[ field ] = value;
            rtnPrt.bestPosn[ field ] = value;
            rtnPrt.velocity[ field ] = 0f;
        }
        rtnPrt.position[ "score" ] = float.NaN;
        rtnPrt.bestPosn[ "score" ] = float.NaN;
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
        
        return bestPrtcl.position;
    }


}

}