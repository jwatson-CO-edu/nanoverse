using OpenTK.Mathematics;
using geo3d;
using parametric;



namespace ribbon;

/* ////////// DEV_PLAN /////////////////////////////////////////////////////////////////////////////
* No root, Only growth points
[Y] Choose a camera angle, Negative Z above <0,0>
[Y] Establish Grid

* Generate Glyph
    
    * Gen Params
        [Y] Prefer moving in the current layer
        [Y] Max number of edges per node
        [Y] Max number of strokes
        [Y] Chance to halt early

    [>] Start with straight lines only
    [ ] Add Quadratic Bezier
    [ ] Add Cubic Bezier
    [ ] Add Circles

    * Generation Steps
        [Y] Initialize 2 Nodes
        * Loop
            [Y] Choose a start node
                [Y] Choose node -or- stroke
            [Y] Choose whether to end on a new node
                [Y] PREVENT vertically stacking nodes 
                [Y] Choose Open grid -or- Along stroke
                [Y] Choose whether to layer jump
            [Y] If new node above then edge to new, Otherwise edge to existing
            [>] Choose edge curve
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
/// Create a layered glyph
/// </summary>
public class GlyphGen ( int NgridHalf = 5, float unitGridSize = Constants._DEFAULT_GRID ) {

    // Structure //
    public List<RNode>  nodes   = []; // Junctions, Both occupied and empty
    public List<Ribbon> strokes = []; // Ribbons that make up the figure

    // Generation //
    public Random rand     = new(); // Local RNG
    public float  haltProb = 0; // --- Probability of halting at the end of each iteration
    public float  nodeProb = 0.5f; //- Probability of beginning at an existing node -vs- creating node on stroke
    public float  newNProb = 0.5f; //- Probability of ending at a new node -vs- using existing

    // Grid Params //
    public Vector3 gridCntr  = Vector3.Zero; // Center of the grid
    public float   gridUnit  = unitGridSize; // Grid step size in the XY plane (`_LAYER_SEP` in Z)
    public int     gridLimit = NgridHalf; // -- Number of steps from the center that the grid is allowed to grow

    // Node Params //
    public int   maxNodePop   = 0; //- Maximum stokes that can meet at the same node
    public int   maxStrokes   = 0; //- Maximum total strokes in the glyph
    public float biasStep_rad = 0f; // Radial spacing for strokes that meet at a node 

    // Stroke Params //
    public float   strokeWidth = unitGridSize * 0.75f; // Width of all strokes
    public Vector4 strokeColor = new(1,1,1,1); // ----------------- Color of all strokes
    public Vector4 borderColor = new(0,0,0,1); // ----------------- Color of all borders (choose BG color!)
    public float   jumpScale   = NgridHalf * unitGridSize; // ----- Scale for computing Z jump probability
    public int     jumpLimit   = 2; // ---------------------------- Max levels that a new node can be moved during Z jump


    /// <summary>
    /// Choose a preferred radial spacing for strokes that meet at a node, Set max node population
    /// </summary>
    public void Init(){
        maxNodePop   = 4 + rand.Next(5);
        biasStep_rad = 2f*MathF.PI / maxNodePop;  
        maxStrokes   = Math.Max( gridLimit, (int) MathF.Pow( 2*gridLimit+1, 2f)/(2*gridLimit) );
        haltProb     = 1f / maxStrokes; 
        nodes.Clear();
        strokes.Clear();
    }


    /// <summary>
    /// Get preferred rays from this node
    /// </summary>
    public List<Vector3> GetBiasRays( RNode node ){
        List<Vector3> rays = [];
        rays.Capacity = maxNodePop;
        for( int i = 0; i < maxNodePop; ++i ){
            rays.Add( MathVec3.AxisAngleQuat( Vector3.UnitZ, i * biasStep_rad ) * node.bias );
        }
        return rays;
    }


    // /// <summary>
    // /// Get the amount of radians away from opposite of the two vectors
    // /// </summary>
    // public static float GetAlignment( Vector3 v1, Vector3 v2 ){
    //     return MathF.PI - MathVec3.AngleBetween( v1, v2 );
    // }


    /// <summary>
    /// Create an address in free space
    /// </summary>
    public int[] ChooseFreeAddress(){
        bool  near;
        int[] posn = new int[2];
        int   iter = 0;
        while( true ){
            posn[0] = rand.Next( gridLimit*2+1 ) - gridLimit;
            posn[1] = rand.Next( gridLimit*2+1 ) - gridLimit;
            near = false;
            foreach( RNode node in nodes ){
                if( node.IsCloseTo( posn ) ){
                    near = true;
                    break;
                }
            }
            if( !near ){  break;  }
            ++iter;
            if( iter > 50 ){  break;  }
        }
        return posn;
    }


    /// <summary>
    /// Create a node in free space
    /// </summary>
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
        foreach( RNode node in nodes ){  if( node.Occupancy() < maxNodePop ){  available.Add( node );  }  }
        return available;
    }


    /// <summary>
    /// Return a list of strokes that are not saturated with (mid-line) connections
    /// </summary>
    public List<Ribbon> AvailableStrokes(){
        List<Ribbon> available = [];
        foreach( Ribbon line in strokes ){  if( line.edges < maxNodePop ){  available.Add( line );  }  }
        return available;
    }


    public RNode PlaceNodeOnStroke( Ribbon stroke ){
        RNode rtnNode;
        float t = rand.NextSingle();
        Vector3 nuBias;
        if( rand.NextSingle() < 0.5 ){
            if( rand.NextSingle() < 0.5 ){  
                nuBias = stroke.spine.Tan(t);  
            }else{
                nuBias = -stroke.spine.Tan(t);
            }
        }else{  nuBias = Vector3.UnitY;  }
        rtnNode = new(){  
            posn = stroke.spine.Val(t),
            bias = nuBias
        };
        return rtnNode;
    }


    /// <summary>
    /// Generate a layered glyph in the form of a triangle mesh
    /// </summary>
    public List<Tri> MakeGlyph(){
        List<Tri> mesh = [];
        
        // Restart
        Init();
        
        // Init 2 nodes
        for( int i = 0; i < 2; ++i ){  nodes.Add( GetFreeNode() );  }

        RNode bgnNode, endNode;
        Ribbon ribbon;
        // Glyph Generation Loop
        while( true ){

            /// Choose Starting Node ///

            // Start from a node
            if( rand.NextSingle() < nodeProb ){
                List<RNode> availN = AvailableNodes();
                if( availN.Count == 0 ){  break;  }
                bgnNode = availN[ rand.Next( availN.Count ) ];

            // Start from a stroke
            }else{
                List<Ribbon> availS = AvailableStrokes();
                if( availS.Count == 0 ){  continue;  }
                Ribbon bgnStroke = availS[ rand.Next( availS.Count ) ];
                bgnNode = PlaceNodeOnStroke( bgnStroke );
                nodes.Add( bgnNode );
            }

            /// Choose Ending Node ///
            
            // End at a new node
            if( rand.NextSingle() < newNProb ){
                endNode = GetFreeNode();
                nodes.Add( endNode );
            
            // End at an existing node
            }else{
                List<RNode> availN = AvailableNodes();
                if( availN.Count == 0 ){  break;  }
                endNode = availN[ rand.Next( availN.Count ) ];
                while( endNode.IsCloseTo( bgnNode) ){  endNode = availN[ rand.Next( availN.Count ) ];  }
            }

            /// Create Stroke ///
            if( rand.NextSingle() < 0.5f ){ 
                ribbon = new( twist_ : 0 ){
                    spine = new Line.Segment( bgnNode.posn, endNode.posn )
                };
                ribbon.SetXdirAt0( Vector3.Cross( Vector3.UnitZ, bgnNode.posn - endNode.posn ).Normalized() );
                ribbon.BuildGeo( strokeWidth );
                ribbon.SetColor( strokeColor, borderColor );
                strokes.Add( ribbon );
            }else{
                float aMin = 6e10f;
                float a;
                float scale = (bgnNode.posn - endNode.posn).Length;
                Vector3 vMin = Vector3.UnitY;
                foreach( Vector3 dir in GetBiasRays( bgnNode ) ){
                    a = MathVec3.AngleBetween( dir, endNode.posn - bgnNode.posn );
                    if( a < aMin ){
                        aMin = a;
                        vMin = dir;
                    }
                } 
                ribbon = new( twist_ : 0 ){
                    spine = new Bezier.Quad( bgnNode.posn, bgnNode.posn + vMin * (scale * rand.NextSingle()), endNode.posn )
                };
                ribbon.SetXdirAt0( Vector3.Cross( Vector3.UnitZ, bgnNode.posn - endNode.posn ).Normalized() );
                ribbon.BuildGeo( strokeWidth );
                ribbon.SetColor( strokeColor, borderColor );
                strokes.Add( ribbon );
            }
            bgnNode.edges.Add( endNode.posn - bgnNode.posn );
            endNode.edges.Add( bgnNode.posn - endNode.posn );

            /// End Conditions ///
            if( strokes.Count >= maxStrokes ){  break;  }
            if( strokes.Count >= maxNodePop ){  if( rand.NextSingle() < haltProb ){  break;  }  }
        }

        Elements elem = new();
        // Aggregate Mesh across Strokes and Nodes
        foreach( Ribbon stroke in strokes ){  mesh.AddRange( stroke.GetTotalMesh() );  }
        foreach( RNode node in nodes ){
            if( node.Occupancy() > 1 ){
                mesh.AddRange( elem.CircleCap( node.posn, Vector3.UnitZ, strokeWidth/2f, strokeColor, borderColor ) );
            }
        }

        // Return Mesh
        return mesh;
    }

}
