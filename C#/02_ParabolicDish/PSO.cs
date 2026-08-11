namespace pso {

/*
using System;
using System.Reflection;
using System.Collections.Generic;

var data = new Dictionary<string, object>
{
    { "Name", "Bob" },
    { "Age", 25 }
};

// 1. Dynamically create an instance of the target type
Person person = Activator.CreateInstance<Person>();

// 2. Iterate through the dictionary and map values to matching properties
foreach (var kvp in data)
{
    PropertyInfo prop = typeof(Person).GetProperty(kvp.Key);
    if (prop != null && prop.CanWrite)
    {
        // Convert the type safely if needed before assigning
        object value = Convert.ChangeType(kvp.Value, prop.PropertyType);
        prop.SetValue(person, value);
    }
}
*/

/// <summary>
/// Particle Swarm Optimization
/// </summary>
public class PSOptimizer<T> {

    public List<string> /*--------------------*/ fields    = [];
    public Dictionary<string,float[]> /*------*/ ranges    = [];
    public LinkedList<Dictionary<string,float>>  particles = [];
    public Dictionary<string,float> /*--------*/ bestPrtcl = [];
    public Dictionary<string,float> /*--------*/ particle  = [];
    public Func<Dictionary<string,float>,float>? Score     = null;
    public Random /*--------------------------*/ rand /**/ = new();

    
    /// <summary>
    /// Add a list of fields
    /// </summary>
    public void AddFields( params string[] fieldsArr ){  fields.AddRange( fieldsArr );  }


    /// <summary>
    /// Set range [lo,hi,step,] for one field
    /// </summary>
    public void AddDomainDiv( string field, float lo, float hi, int div = 10 ){
        if( fields.Contains( field ) ){
            if( lo > hi ){  throw new Exception( $"Domain [{lo}, {hi}] is MALFORMED!" );  }
            ranges[ field ] = [lo, hi, div, (hi-lo)/div,];
        }
        throw new Exception( $"Key {field} DNE for this problem!" );
    }


    /// <summary>
    /// Set function that will be used for scoring each particle
    /// </summary>
    public void SetScoringFunc( Func<Dictionary<string,float>,float>? func ){  Score = func;  }


    public Dictionary<string,float> RandParticle(){
        Dictionary<string,float> rtnPrt = [];
        foreach( string field in fields ){
            rtnPrt[ field ] = ranges[ field ][0] + rand.Next( (int) ranges[ field ][2] + 1 ) * ranges[ field ][3];
        }
        rtnPrt[ "score" ] = float.NaN;
        return rtnPrt;
    }


    public void PopulateInit( int N = 1000 ){
        for( int i = 0; i < N; ++i ){  particles.AddLast( RandParticle() );  }
    }





}

}