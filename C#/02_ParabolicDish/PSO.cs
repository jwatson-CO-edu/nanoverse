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
    /// Multiply vector with keyed components by a `factor`
    /// </summary>
    public static DctVecF operator*( float factor, DctVecF v1 ){
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


    /// <summary>
    /// Return a copy of this vector
    /// </summary>
    public DctVecF Copy(){
        DctVecF rtnVec = [];
        foreach( (string key, float val) in this ){  rtnVec[key] = val;  }
        return rtnVec;
    }


    /// <summary>
    /// Human-readable representation of this vector's keyed components,
    /// SLOP: https://claude.ai/share/a78d4ef7-7976-4656-a610-f9c013ea41bd
    /// </summary>
    public override string ToString(){
        List<string> parts = [];
        foreach( (string key, float val) in this ){
            if( !skip.Contains( key ) ){  parts.Add( $"{key}: {val:F4}" );  }
        }
        string rtnStr = string.Join( ", ", parts );
        if( ContainsKey( "score" ) ){  rtnStr += $" | score: {this["score"]:F4}";  }
        return "{ " + rtnStr + " }";
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

    
    /// <summary>
    /// Change particle `position` according to the `velocity`
    /// </summary>
    public void Advance(){  position += velocity;  }
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
        fields.Add( field );
        ranges[ field ] = [lo, hi, div, (hi-lo)/div,];
    }


    /// <summary>
    /// Set function that will be used for scoring each particle
    /// </summary>
    public void SetScoringFunc( Func<DctVecF,float>? func ){  Score = func;  }


    /// <summary>
    /// Generate a random particle at a grid point
    /// </summary>
    public Particle RandGridParticle(){
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
    /// Random `DctVecF` within problem bounds
    /// </summary>
    public DctVecF RandVector(){
        DctVecF vec = [];
        float   val;
        foreach( string field in fields ){
            val = ranges[ field ][0] + (ranges[ field ][1] - ranges[ field ][0]) * rand.NextSingle();
            vec[ field ] = val;
        }
        vec[ "score" ] = float.NaN;
        return vec;
    }


     /// <summary>
    /// Random `DctVecF` within problem bounds
    /// </summary>
    public DctVecF ZeroVector(){
        DctVecF vec = [];
        foreach( string field in fields ){
            vec[ field ] = 0f;
        }
        vec[ "score" ] = float.NaN;
        return vec;
    }


    /// <summary>
    /// Prevent particle from being kicked outside of the problem domain!
    /// </summary>
    public void EnforceBounds( Particle prtcl ){
        foreach( string field in fields ){
            prtcl.position[ field ] = Math.Clamp( prtcl.position[ field ], ranges[ field ][0], ranges[ field ][1] );
        }
    }


    /// <summary>
    /// Caclulate fitness of particle position
    /// </summary>
    public void EvalParticlePosn( Particle prtcl ){
        if( Score is null ){  throw new Exception( "`Score` function is UNDEFINED!" );  }
        prtcl.position["score"] = Score( prtcl.position );
    }


    /// <summary>
    /// Caclulate fitness of particle position and update best position if better
    /// </summary>
    public void UpdateParticleBest( Particle prtcl ){
        if( Score is null ){  throw new Exception( "`Score` function is UNDEFINED!" );  }
        EvalParticlePosn( prtcl );
        if( prtcl.position["score"] > prtcl.bestPosn["score"] ){  prtcl.bestPosn = prtcl.position.Copy();  }
    }


    /// <summary>
    /// Generate N random particles at grid points, Called by client BEFORE `Solve`
    /// </summary>
    public void PopulateInit( int N = 0 ){
        if( N < 1 ){  
            N = 1;   
            foreach( string field in fields ){  
                N *= (int) ranges[ field ][2];  
                Console.WriteLine( $"\tAbout to init {N} particles!" );
            }
            N /= 2;
        }
        particles.Clear();
        Console.WriteLine( $"About to init {N} particles!" );
        for( int i = 0; i < N; ++i ){  particles.AddLast( RandGridParticle() );  }
        bestPrtcl = RandGridParticle();
    }


    /// <summary>
    /// Average speed of the swarm, Used to detect convergence
    /// </summary>
    public float AverageSwarmSpeed(){
        float velAvg = 0;
        foreach( Particle prtcl in particles ){  velAvg += prtcl.velocity.Length();  }
        Console.WriteLine( $"Total vel {velAvg}" );
        return velAvg / particles.Count;
    }


    /// <summary>
    /// Search across swarm for the current best solution 
    /// </summary>
    public void UpdateSwarmBest(){
        foreach( Particle prtcl in particles ){    
            if( prtcl.position["score"] > bestPrtcl.position["score"] ){  bestPrtcl = prtcl;  }
        }
    }


    /// <summary>
    /// Run PSO until max iterations or min speed (convergence) is reached,
    /// Source: https://en.wikipedia.org/wiki/Particle_swarm_optimization#Algorithm
    /// </summary>
    public DctVecF Solve( int N = 2000, float haltFrac = 0.01f ){

        EvalParticlePosn( bestPrtcl );

        if( Score is null ){  throw new Exception( "`Score` function is UNDEFINED!" );  }

        /// Stage 0: Init problem scale ///
        float scale  = 0f;
        float width  = 0f;
        float prtScl;
        float initSpd;
        float lastSpd;
        float randTemp = 1.0f;
        float veloTemp = 1.0f;
        float coolRate = 1f / N;
        float frac;
        float momentum = 0.5f;
        DctVecF vel_ij;
        foreach( string field in fields ){  
            scale += MathF.Pow( ranges[ field ][3], 2f );  
            width += MathF.Pow( ranges[ field ][1] - ranges[ field ][0], 2f );  
        }
        scale = MathF.Sqrt( scale );
        width = MathF.Sqrt( width );
        float factor = scale / width;

        /// Stage 1: Init particle velocities ///
        foreach( Particle prtcl in particles ){
            if( rand.Next( 10 ) == 1 ){  Console.Write( "." );  }
            particle = RandGridParticle();
            prtScl   = MathF.Min( (prtcl.position - particle.position).Length(), scale );
            Console.WriteLine( $"{(prtcl.position - particle.position).Length()}, {scale}" );
            if( Score( prtcl.position ) > Score( particle.position ) ){
                prtcl.velocity = (prtcl.position - particle.position).Normalized() * prtScl;
                prtcl.bestPosn = prtcl.position.Copy();
                prtcl.bestPosn["score"] = Score( prtcl.bestPosn );
                prtcl.position = particle.position;
                prtcl.position["score"] = Score( prtcl.position );
            }else{
                prtcl.velocity = (particle.position - prtcl.position).Normalized() * prtScl;
                prtcl.position["score"] = Score( prtcl.position );
                prtcl.bestPosn = particle.position.Copy();
                prtcl.bestPosn["score"] = Score( prtcl.bestPosn );
            }
        }
        initSpd = 1f;
        lastSpd = 1f;
        
        /// Stage 2: Search ///
        int count = 0;

        Console.WriteLine( $"About to iterate ..." );
        Console.WriteLine( $"({count} < {N}) && (({lastSpd} / {initSpd}) > {haltFrac})" );


        while( (count < N) && ((lastSpd / initSpd) > haltFrac) ){

            Console.WriteLine( $"Iteration {count+1} " );

            foreach( Particle prtcl in particles ){
                frac   = rand.NextSingle(); // Random blend of global / personal best
                vel_ij = ZeroVector();

                // Fields can be DIFFERENT scales, So treat each individually!
                foreach( string field in fields ){
                    factor = 1f / ranges[ field ][2];
                    vel_ij[ field ] = 
                        (momentum * prtcl.velocity[ field ]) + // Momentum is important for convergence!
                        (1f-momentum) * (
                            veloTemp * factor * (1f - frac) * (bestPrtcl.bestPosn[ field ] - prtcl.position[ field ]) + // Seeking global best
                            veloTemp * factor * frac * (prtcl.bestPosn[ field ] - prtcl.position[ field ]) + // ---------- Seeking personal best
                            randTemp * ranges[ field ][3] * rand.NextSingle() // ------------------------------ Seeking cooled random vector
                        );
                }
                         
                prtcl.velocity = vel_ij;
                prtcl.Advance();
                EnforceBounds( prtcl ); // Keep particles from being kicked outside of the problem domain!
                UpdateParticleBest( prtcl );
            }
            UpdateSwarmBest();

            randTemp = MathF.Max( 0F, randTemp-coolRate*1.5f );
            veloTemp = MathF.Max( 0F, veloTemp-coolRate*0.75f );
            lastSpd  = AverageSwarmSpeed();
            if( count == 0 ){  initSpd = lastSpd;  }
            ++count;
        }
        
        return bestPrtcl.position;
    }


}

}