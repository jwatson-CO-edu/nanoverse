using OpenTK.Mathematics;
using parametric;
using geo3d;
using pose3d;

namespace ribbon {



/// <summary>
/// Useful `Ribbon` Constants
/// </summary>
public class Constants {
    public const int   _DEFAULT_DIV  = 64;
    public const float _PLY_SEP      = 1f / 32f; //1f / 64f;
    public const float _LAYER_SEP    = 3f * _PLY_SEP; //1f / 64f;
    public const float _DEFAULT_GRID = 0.5f; //1f / 64f;
}



/// <summary>
/// Double-sided flat stroke that sweeps through space with a border
/// </summary>
public class Ribbon {
    
    public int /*-----*/ div; // -- Number of chunks that define the ribbon
    public int /*-----*/ N; // ---- Number of boundaries that define the ribbon
    public float /*---*/ twist; //- Radians of twist along `t` = [0,1]
    public float /*---*/ dTheta; // Twist along each chunk  
    public Parametric    spine; //- Curve at the center of the ribbon
    public float /*---*/ dt; // --- Parametric length of each chunk
    public List<float>   tLst; // - Parameter of chunk borders
    public List<float>   Xwdt; // - Width along X at chunk borders
    public List<Matrix4> matx; // - Width direction at chunk borders
    public List<Tri>     top; // -- Top ribbon
    public List<Tri>     mid; // -- Middle border
    public List<Tri>     btm; // -- Bottom ribbon
    public bool /*----*/ hasMid; // Flag: Should the middle layer be constructed?
    public int /*-----*/ edges; //- Graph structure, Unused?


    /*     Y  +X ====================
           | /                   
           +--- Z    --RIBBON->   
         /
       -X ====================     */
    
    
    /// <summary>
    /// Default Constructor 
    /// </summary>
    public Ribbon ( int div_ = Constants._DEFAULT_DIV, float twist_ = 0f, bool makeMid = true ){
        // Setup Params
        div    = div_;
        N /**/ = div_+1;
        twist  = twist_;
        dt     = 1.0f / div;
        dTheta = twist / div;
        spine  = new DummyCurve();
        tLst   = [];
        Xwdt   = [];
        matx   = [];
        top    = [];
        mid    = [];
        btm    = [];
        hasMid = makeMid;
        edges  = 0;
        // Reserve Geo
        matx.Capacity = N;
        tLst.Capacity = N;
        Xwdt.Capacity = N;
        top.Capacity  = 2*div;
        mid.Capacity  = 4*div;
        btm.Capacity  = 2*div;        
        for( int i = 0; i <= div; ++i ){  tLst.Add( i * dt ); }
    }


    /// <summary>
    /// Get the index of the segment on which `t` lies 
    /// </summary>
    public int GetSegment( float t ){
        for( int i = 0; i < N; ++i ){  if( tLst[i] > t ){  return i-1;  }  }
        return N-1;
    }


    /// <summary>
    /// Get the control frame for the segment that contains `t` 
    /// </summary>
    public Matrix4 GetFrameAt( float t ){  return matx[ GetSegment(t) ];  }


    /// <summary>
    /// Set the twist across the curve 
    /// </summary>
    public void SetTwist( float twist_ = 0f ){
        twist  = twist_;
        dTheta = twist / div;
    }


    /// <summary>
    /// Set the `Ribbon` orientation at t=0 
    /// </summary>
    public void SetXdirAt0( Vector3 xDir, bool compute = true ){
        Vector3 zDir;
        matx.Clear();
        matx.Capacity = N;
        zDir = spine.Tan(0).Normalized();
        matx.Add( MathMatx4.HomogFromXZBases( xDir, zDir, spine.Val(0) ) );
        if( compute ){  ComputeMatrices();  }
    }


    /// <summary>
    /// Set the `Ribbon` orientation at t>0, ASSUMPTION: `SetXdirAt0()` has already been called!
    /// </summary>
    public void ComputeMatrices(){
        Vector3 Z_im1, X_im1, Z_i, X_i;
        for( int i = 1; i < N; ++i ){
            X_im1 = MathMatx4.GetXBasis( matx[i-1] );
            Z_im1 = MathMatx4.GetZBasis( matx[i-1] );
            X_i   = MathVec3.AxisAngleQuat( Z_im1, dTheta ) * X_im1;
            Z_i   = spine.Tan( tLst[i] );
            matx.Add( MathMatx4.HomogFromXZBases( X_i, Z_i, spine.Val( tLst[i] ) ) );
        }
    }


    /// <summary>
    /// Create meshes for drawing. 
    /// </summary>
    public void BuildGeo( float width1, float width2 = 0f ){
        float t_i, t_ip1, w_i, w_ip1, h_i, h_ip1;
        if( width2 < 0.00001 ){  width2 = width1;  }
        
        float dW /**/ = (width2 - width1) / div;
        float margin  = Math.Max( width1, width2 ) / 8f;
        float[] width = new float[N];

        for( int i = 0; i < N; ++i ){  width[i] = width1 + i * dW;  }
        
        Vector3 X_i, X_ip1, Y_i, mid_i, mid_ip1, pt0, pt1, pt2, pt3;
        
        for( int i = 0; i < N-1; ++i ){
            t_i     = tLst[i];
            t_ip1   = tLst[i+1];
            w_i     = width[i];
            w_ip1   = width[i+1];
            h_i     = w_i/2f;
            h_ip1   = w_ip1/2f;
            X_i     = MathMatx4.GetXBasis( matx[i] );
            X_ip1   = MathMatx4.GetXBasis( matx[i+1] );
            Y_i     = MathMatx4.GetYBasis( matx[i] );
            mid_i   = spine.Val( t_i   );
            mid_ip1 = spine.Val( t_ip1 );

            /// Middle (Border) Geometry ///
            pt0 = mid_i   + X_i   * (h_i   + margin);
            pt1 = mid_i   - X_i   * (h_i   + margin);
            pt2 = mid_ip1 + X_ip1 * (h_ip1 + margin);
            pt3 = mid_ip1 - X_ip1 * (h_ip1 + margin);

            // Top Middle // 
            mid.Add( new Tri( pt1, pt0, pt3 ) ); 
            mid.Add( new Tri( pt2, pt3, pt0 ) );
            // Bottom Middle //
            mid.Add( new Tri( pt3, pt0, pt1 ) ); 
            mid.Add( new Tri( pt3, pt2, pt0 ) );

            /// Top (Visual) Geometry ///
            pt0 = mid_i   + X_i   * h_i   + Y_i * Constants._PLY_SEP;
            pt1 = mid_i   - X_i   * h_i   + Y_i * Constants._PLY_SEP;
            pt2 = mid_ip1 + X_ip1 * h_ip1 + Y_i * Constants._PLY_SEP;
            pt3 = mid_ip1 - X_ip1 * h_ip1 + Y_i * Constants._PLY_SEP;

            top.Add( new Tri( pt1, pt0, pt3 ) ); 
            top.Add( new Tri( pt2, pt3, pt0 ) );

            /// Bottom (Visual) Geometry ///
            pt0 = mid_i   + X_i   * h_i   - Y_i * Constants._PLY_SEP;
            pt1 = mid_i   - X_i   * h_i   - Y_i * Constants._PLY_SEP;
            pt2 = mid_ip1 + X_ip1 * h_ip1 - Y_i * Constants._PLY_SEP;
            pt3 = mid_ip1 - X_ip1 * h_ip1 - Y_i * Constants._PLY_SEP;

            btm.Add( new Tri( pt3, pt0, pt1 ) ); 
            btm.Add( new Tri( pt3, pt2, pt0 ) );
        }
    }
    

    /// <summary>
    /// Set color meshes 
    /// </summary>
    public void SetColor( Vector4 stroke, Vector4 border ){
        top = Tri.ColorMesh( top, stroke );
        btm = Tri.ColorMesh( btm, stroke );
        mid = Tri.ColorMesh( mid, border );
    }


    /// <summary>
    /// Return the aggregate mesh
    /// </summary>
    public List<Tri> GetTotalMesh(){
        List<Tri> rtnMsh = [];
        rtnMsh.Capacity = top.Count + mid.Count + btm.Count;
        rtnMsh.AddRange( top );
        rtnMsh.AddRange( mid );
        rtnMsh.AddRange( btm );
        return rtnMsh;
    }

}

/// <summary>
/// Ribbon-like elements other than curves
/// </summary>
public class Elements {

    public Random rand = new();

    /// <summary>
    /// Rounded stroke cap
    /// </summary>
    public List<Tri> CircleCap( Vector3 center, Vector3 normal, float radius, Vector4 strokeColor, Vector4 borderColor, int div = 16 ){
        List<Tri> circ = [];
        List<Tri> top  = [];
        List<Tri> mid  = [];
        List<Tri> btm  = [];
        circ.Capacity = div * 4;
        mid.Capacity = div * 2;
        top.Capacity = div;
        btm.Capacity = div;
        normal.Normalize();
        float   dTheta = 2f * MathF.PI / div;
        float   margin = radius / 4f;
        Vector3 spar   = Vector3.Cross( MathVec3.NoiseXYZ( rand ), normal ).Normalized();
        Vector3 p0, p1, p2;

        for( int i = 0; i < div; ++i ){

            p0 = center + normal * Constants._PLY_SEP;
            p1 = center + MathVec3.AxisAngleQuat( normal, i     * dTheta ) * spar * radius + normal * Constants._PLY_SEP;
            p2 = center + MathVec3.AxisAngleQuat( normal, (i+1) * dTheta ) * spar * radius + normal * Constants._PLY_SEP;
            top.Add( new Tri( p0, p2, p1 ) );

            p0 = center;
            p1 = center + MathVec3.AxisAngleQuat( normal, i     * dTheta ) * spar * (radius + margin);
            p2 = center + MathVec3.AxisAngleQuat( normal, (i+1) * dTheta ) * spar * (radius + margin);
            mid.Add( new Tri( p0, p2, p1 ) );

            p1 = center + MathVec3.AxisAngleQuat( -normal, i     * dTheta ) * spar * (radius + margin);
            p2 = center + MathVec3.AxisAngleQuat( -normal, (i+1) * dTheta ) * spar * (radius + margin);
            mid.Add( new Tri( p0, p2, p1 ) );

            p0 = center - normal * Constants._PLY_SEP;
            p1 = center + MathVec3.AxisAngleQuat( -normal, i     * dTheta ) * spar * radius - normal * Constants._PLY_SEP;
            p2 = center + MathVec3.AxisAngleQuat( -normal, (i+1) * dTheta ) * spar * radius - normal * Constants._PLY_SEP;
            btm.Add( new Tri( p0, p2, p1 ) );
        }

        circ.AddRange( Tri.ColorMesh( top, strokeColor ) );        
        circ.AddRange( Tri.ColorMesh( btm, strokeColor ) );        
        circ.AddRange( Tri.ColorMesh( mid, borderColor ) );
        return circ;
    }


    /// <summary>
    /// Square stroke cap
    /// </summary>
    public static List<Tri> SquareCap( Vector3 center, Vector3 normal, Vector3 align, float sideLength, Vector4 strokeColor, Vector4 borderColor, int div = 16 ){
        List<Tri> squ = [];
        List<Tri> top = [];
        List<Tri> mid = [];
        List<Tri> btm = [];
        normal.Normalize();
        align.Normalize();
        Vector3 yDir = Vector3.Cross( normal, align ).Normalized();
        Vector3 p0, p1, p2, p3;
        float   half   = sideLength/2f;
        float   margin = sideLength/8f;

        p0 = center + align * (half+margin) + yDir * (half+margin);
        p1 = center - align * (half+margin) + yDir * (half+margin);
        p2 = center - align * (half+margin) - yDir * (half+margin);
        p3 = center + align * (half+margin) - yDir * (half+margin);

        mid.Add( new Tri( center, p0, p1 ) );
        mid.Add( new Tri( center, p1, p2 ) );
        mid.Add( new Tri( center, p2, p3 ) );
        mid.Add( new Tri( center, p3, p0 ) );

        mid.Add( new Tri( center, p1, p0 ) );
        mid.Add( new Tri( center, p2, p1 ) );
        mid.Add( new Tri( center, p3, p2 ) );
        mid.Add( new Tri( center, p0, p3 ) );

        p0 = center + align * half + yDir * half + normal * Constants._PLY_SEP;
        p1 = center - align * half + yDir * half + normal * Constants._PLY_SEP;
        p2 = center - align * half - yDir * half + normal * Constants._PLY_SEP;
        p3 = center + align * half - yDir * half + normal * Constants._PLY_SEP;

        top.Add( new Tri( center, p0, p1 ) );
        top.Add( new Tri( center, p1, p2 ) );
        top.Add( new Tri( center, p2, p3 ) );
        top.Add( new Tri( center, p3, p0 ) );

        p0 = center + align * half + yDir * half - normal * Constants._PLY_SEP;
        p1 = center - align * half + yDir * half - normal * Constants._PLY_SEP;
        p2 = center - align * half - yDir * half - normal * Constants._PLY_SEP;
        p3 = center + align * half - yDir * half - normal * Constants._PLY_SEP;

        btm.Add( new Tri( center, p1, p0 ) );
        btm.Add( new Tri( center, p2, p1 ) );
        btm.Add( new Tri( center, p3, p2 ) );
        btm.Add( new Tri( center, p0, p3 ) );

        squ.AddRange( Tri.ColorMesh( top, strokeColor ) );        
        squ.AddRange( Tri.ColorMesh( btm, strokeColor ) );        
        squ.AddRange( Tri.ColorMesh( mid, borderColor ) );
        return squ;
    }
}

/* ////////// DEV_PLAN /////////////////////////////////////////////////////////////////////////////
* No root, Only growth points
[Y] Choose a camera angle, Negative Z above <0,0>
[Y] Establish Grid

* Generate Glyph
    
    * Gen Params
        [ ] Prefer moving in the current layer
        [ ] Max number of edges per node
        [ ] Max number of strokes
        [ ] Chance to halt early

    [ ] Start with straight lines only
    [ ] Add Quadratic Bezier
    [ ] Add Cubic Bezier
    [ ] Add Circles

    * Generation Steps
        [ ] Initialize 2 Nodes
        * Loop
            [ ] Choose a start node
                [ ] Choose node -or- stroke
            [ ] Choose whether to generate a new node
                [ ] PREVENT vertically stacking nodes 
                [ ] Choose Open grid -or- Along stroke
                [ ] Choose whether to layer jump
            
            [ ] If new node above then edge to new, Otherwise edge to existing
            [ ] Choose edge curve
        [ ] Finish Nodes
            [ ] Terminate stroke ends (one edge)
            [ ] Add circles to nodes w multiple edges

[Y] `Elements`
    [Y] Circle corner
    [Y] Square corner
[ ] Disable lighting
[ ] Output image
*/


/// <summary>
/// Model junction between `Ribbon`s
/// </summary>
public class RNode {
    public int[] /*---*/ addr    = new int[2];
    public Vector3 /*-*/ posn    = Vector3.Zero; // Anchor Point
    public List<Vector3> edges   = []; // --------- Strokes leaving this node
    public Vector3 /*-*/ bias    = Vector3.Zero; // Bias direction (Central bias ray)
    public float /*---*/ dThresh = Constants._DEFAULT_GRID / 4f; 


    /// <summary>
    /// Is this node close to an address?
    /// </summary>
    public bool IsCloseTo( int[] q ){
        return (Math.Abs(addr[0] - q[0]) + Math.Abs(addr[1] - q[1])) == 0;            
    }


    /// <summary>
    /// Is this node close to a location?
    /// </summary>
    public bool IsCloseTo( Vector3 q ){
        return (new Vector2( posn[0], posn[1] ) - new Vector2( q[0], q[1] )).Length <= dThresh;               
    }


    /// <summary>
    /// Distance to a location
    /// </summary>
    public float DistanceTo( Vector3 q ){
        return (new Vector2( posn[0], posn[1] ) - new Vector2( q[0], q[1] )).Length;               
    }


    /// <summary>
    /// Is this node close to another node?
    /// </summary>
    public bool IsCloseTo( RNode q ){
        return (new Vector2( posn[0], posn[1] ) - new Vector2( q.posn[0], q.posn[1] )).Length <= dThresh;               
    }


    /// <summary>
    /// Distance to another node
    /// </summary>
    public float DistanceTo( RNode q ){
        return (new Vector2( posn[0], posn[1] ) - new Vector2( q.posn[0], q.posn[1] )).Length;               
    }


    /// <summary>
    /// Is this node close to any in a list of other nodes?
    /// </summary>
    public bool IsCloseTo( List<RNode> q ){
        foreach( RNode node in q ){  if( IsCloseTo( node ) ){  return true;  }  }
        return false;
    }


    /// <summary>
    /// How many strokes join here?
    /// </summary>
    public int Occupancy(){  return edges.Count;  }
}



/// <summary>
/// Create a layered glyph
/// </summary>
public class GlyphGen ( int NgridHalf = 5, float unitGridSize = Constants._DEFAULT_GRID ) {

    // Generation //
    public Random rand = new();

    // Structure //
    public List<RNode>  nodes   = []; // Junctions, Both occupied and empty
    public List<Ribbon> strokes = []; // Ribbons that make up the figure

    // Grid Params //
    public Vector3 gridCntr  = Vector3.Zero; // Center of the grid
    public float   gridUnit  = unitGridSize; // Grid step size in the XY plane (`_LAYER_SEP` in Z)
    public int     gridLimit = NgridHalf; // -- Number of steps from the center that the grid is allowed to grow

    // Node Params //
    public int   maxPop /*-*/ = 0;
    public float biasStep_rad = 0f; 

    // Stroke Params //
    public float   strokeWidth = Constants._DEFAULT_GRID * 0.4f; // Width of all strokes
    public Vector4 strokeColor = new(1,1,1,1); // ----------------- Color of all strokes
    public Vector4 borderColor = new(0,0,0,1); // ----------------- Color of all borders (choose BG color!)
    public float   jumpScale   = NgridHalf * unitGridSize;
    public int     jumpLimit   = 2;


    /// <summary>
    /// Choose a preferred radial spacing for strokes that meet at a node, Set max node population
    /// </summary>
    public void InitBiasDiv(){
        maxPop /*-*/ = 4 + rand.Next(5);
        biasStep_rad = 2f*MathF.PI / maxPop;  
    }


    /// <summary>
    /// Create a node in free space
    /// </summary>
    public int[] ChooseFreeAddress(){
        bool  near;
        int[] posn = new int[2];
        while( true ){
            posn[0] =  rand.Next( gridLimit*2+1 ) - gridLimit;
            posn[1] =  rand.Next( gridLimit*2+1 ) - gridLimit;
            near = false;
            foreach( RNode node in nodes ){
                if( node.IsCloseTo( posn ) ){
                    near = true;
                    break;
                }
            }
            if( !near ){  break;  }
        }
        return posn;
    }


    public RNode GetFreeNode(){
        RNode rtnNod = new(){  addr = ChooseFreeAddress()  };
        
        rtnNod.posn[0] = gridCntr[0] + rtnNod.addr[0] * gridUnit;
        rtnNod.posn[1] = gridCntr[1] + rtnNod.addr[1] * gridUnit;
        rtnNod.posn[2] = gridCntr[2];

        float dMin = 6e10f;
        float zMin = 6e10f;
        float d;

        // Determine Z-separation from nearest neighbor
        foreach( RNode node in nodes ){
            d = node.DistanceTo( rtnNod );
            if( d < dMin ){
                dMin = d;
                zMin = Math.Abs( node.posn[2] - rtnNod.posn[2] );
            }
        }

        // Compute vertical jump probability and roll to jump up to `jumpLimit` layers 
        if( zMin <= MathVec3._EPSILON ){
            float jumpProb = 1f - Math.Min( 1f, dMin / jumpScale );
            if( rand.NextSingle() < jumpProb ){
                rtnNod.posn[2] += (rand.Next( 2*jumpLimit+1 ) - jumpLimit) * Constants._LAYER_SEP;
            }
        }

        return rtnNod;
    }


    /// <summary>
    /// Return a list of nodes that are not saturated with connections
    /// </summary>
    public List<RNode> AvailableNodes(){
        List<RNode> available = [];
        foreach( RNode node in nodes ){  if( node.Occupancy() < maxPop ){  available.Add( node );  }  }
        return available;
    }


    /// <summary>
    /// Return a list of strokes that are not saturated with (mid-line) connections
    /// </summary>
    public List<Ribbon> AvailableStrokes(){
        List<Ribbon> available = [];
        foreach( Ribbon line in strokes ){  if( line.edges < maxPop ){  available.Add( line );  }  }
        return available;
    }


    /// <summary>
    /// Generate a layered glyph in the form of a triangle mesh
    /// </summary>
    public List<Tri> MakeGlyph(){
        List<Tri> mesh  = [];
        
        // Restart
        nodes.Clear();
        strokes.Clear();
        InitBiasDiv();
        
        // Init 2 nodes
        for( int i = 0; i < 2; ++i ){  nodes.Add( GetFreeNode() );  }

        // Glyph Generation Loop
        while( true ){
            break;
        }

        // Aggregate Mesh across Strokes and Nodes

        // Return Mesh
        return mesh;
    }

}

    
}