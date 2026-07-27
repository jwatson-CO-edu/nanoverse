

namespace sigil {

public class Pictogram ( float thickness ) {

    public const int    _MAX_STROKES = 64; 
    public List<Stroke> strokes /**/ = [];
    public float /*--*/ thick /*--*/ = thickness;
    public float /*--*/ maxDim /*-*/ = 1024;

    public void Generate( int maxStrokes = _MAX_STROKES, float breakProb = 1.0f / (_MAX_STROKES/2) ){
        int maxStrk = maxStrokes;
        int count   = 0;

        strokes.Capacity = maxStrk;
        
        // FIXME: START HERE:
        while( count < maxStrk ){
            // ROLL FOR START LOCATION, SELECT {0, 1, t}
            // ROLL FOR STROKE TYPE
            // ROLL FOR ENDPOINT / PARAMS
            // CREATE STROKE
                // SCAN FOR INTERSECTIONS
                // FOR EACH INTERSECTION: SELECT ONE {CROSS, ABOVE, BELOW}
            // CREATE UNDERSTROKE
            // ROLL FOR BREAK
        }

        // FIXME: FOR EACH STROKE
            // CREATE GEO
                // STROKE
                // UNDERSTROKE
    }

}

}