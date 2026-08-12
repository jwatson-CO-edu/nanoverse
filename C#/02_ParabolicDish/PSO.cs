namespace pso {


////////// DICTIONARY VECTOR ///////////////////////////////////////////////////////////////////////

/// <summary>
/// Vector with keyed components instead of ordered components
/// </summary>
public class DctVecF : Dictionary<string,float> {

    /// Special Keys ///
    public List<string> skip = ["score"];
    
    
    /// <summary>
    /// Add vectors with keyed components
    /// </summary>
    public static DctVecF operator+( DctVecF v1, DctVecF v2 ){
        DctVecF rtnVec = [];
        foreach( (string key, float val1) in v1 ){
            if( !v1.skip.Contains( key ) ){  rtnVec[key] = val1 + v2[key];  }
        }
        return rtnVec;
    }    

    
    /// <summary>
    /// Subtract vectors with keyed components
    /// </summary>
    public static DctVecF operator-( DctVecF v1, DctVecF v2 ){
        DctVecF rtnVec = [];
        foreach( (string key, float val1) in v1 ){
            if( !v1.skip.Contains( key ) ){  rtnVec[key] = val1 - v2[key];  }
        }
        return rtnVec;
    }

    
    /// <summary>
    /// Divide vector with keyed components by a `factor`
    /// </summary>
    public static DctVecF operator/( DctVecF v1, float factor ){
        DctVecF rtnVec = [];
        foreach( (string key, float val1) in v1 ){
            if( !v1.skip.Contains( key ) ){  rtnVec[key] = val1 / factor;  }
        }
        return rtnVec;
    }


    /// <summary>
    /// Multiply vector with keyed components by a `factor`
    /// </summary>
    public static DctVecF operator*( DctVecF v1, float factor ){
        DctVecF rtnVec = [];
        foreach( (string key, float val1) in v1 ){
            if( !v1.skip.Contains( key ) ){  rtnVec[key] = val1 * factor;  }
        }
        return rtnVec;
    }

    
    /// <summary>
    /// Get Euclidian length of all keyed components
    /// </summary>
    public float Length(){
        float length = 0;
        foreach( (string key, float val) in this ){
            if( !skip.Contains( key ) ){  length += MathF.Pow( val, 2 );  }
        }
        return MathF.Sqrt( length );
    }


    /// <summary>
    /// Return a Unit version of this vector
    /// </summary>
    public DctVecF Normalized(){
        float mag = Length();
        if( mag > 0f ){  return this / mag;  }
        return this * 1f;
    }

}



////////// PARTICLE ////////////////////////////////////////////////////////////////////////////////

/// <summary>
/// Base component of Particle Swarm Optimization
/// </summary>
public class Particle {
    public DctVecF position = [];
    public DctVecF velocity = [];
    public DctVecF bestPosn = [];
}



////////// PSO /////////////////////////////////////////////////////////////////////////////////////

/// <summary>
/// Particle Swarm Optimization
/// </summary>
public class PSOptimizer {

    public List<string> /*---------*/ fields    = [];
    public Dictionary<string,float[]> ranges    = [];
    public LinkedList<Particle> /*-*/ particles = [];
    public Particle /*-------------*/ bestPrtcl = new();
    public Particle /*-------------*/ particle  = new();
    public Func<DctVecF,float>? /*-*/ Score     = null;
    public Random /*---------------*/ rand /**/ = new();

    
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
    public void SetScoringFunc( Func<DctVecF,float>? func ){  Score = func;  }


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
        if( Score is not null ){
            rtnPrt.position[ "score" ] = Score( rtnPrt.position );
            rtnPrt.bestPosn[ "score" ] = Score( rtnPrt.bestPosn );
        }else{
            rtnPrt.position[ "score" ] = float.NaN;
            rtnPrt.bestPosn[ "score" ] = float.NaN;
        }
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


    public float AverageSpeed(){
        float velAvg = 0;
        foreach( Particle prtcl in particles ){  velAvg += prtcl.velocity.Length();  }
        return velAvg / particles.Count;
    }


    /// <summary>
    /// Run PSO until max iterations or min speed is reached,
    /// Source: https://en.wikipedia.org/wiki/Particle_swarm_optimization#Algorithm
    /// </summary>
    public DctVecF Solve( int N = 2000, float haltFrac = 0.01f ){

        if( Score is null ){  throw new Exception( "`Score` function is UNDEFINED!" );  }

        /// Stage 0: Init problem scale ///
        float    scale  = 0f;
        float    prtScl;
        float    initSpd;
        float    lastSpd;
        foreach( string field in fields ){  scale += MathF.Pow( ranges[ field ][3], 2f );  }
        scale = MathF.Sqrt( scale );

        /// Stage 1: Init particle velocities ///
        foreach( Particle prtcl in particles ){
            particle = RandParticle();
            prtScl   = MathF.Min( (prtcl.position - particle.position).Length(), scale );
            if( Score( prtcl.position ) > Score( particle.position ) ){
                prtcl.velocity = (prtcl.position - particle.position).Normalized() * prtScl;
                prtcl.position = particle.position;
            }else{
                prtcl.velocity = (particle.position - prtcl.position).Normalized() * prtScl;
            }
        }
        initSpd = AverageSpeed();
        lastSpd = initSpd;
        
        /// Stage 2: Search ///
        int count = 0;
        while( (count < N) && ((lastSpd / initSpd) > haltFrac) ){
            // FIXME: START HERE
            
            lastSpd = AverageSpeed();
            ++count;
        }
        
        return bestPrtcl.position;
    }


}

}