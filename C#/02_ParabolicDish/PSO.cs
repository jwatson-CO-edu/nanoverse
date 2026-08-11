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

    public List<string> /*---------*/ fields    = [];
    public Dictionary<string,float[]> ranges    = [];
    public LinkedList<T> /*--------*/ particles = [];
    public T? /*-------------------*/ bestPrtcl = default;

    
    public void AddFields( params string[] fieldsArr ){
        fields.AddRange( fieldsArr );
    }


    public void AddRange( string field, float[] range ){
        if( fields.Contains( field ) ){
            if( range.Length < 2 ){ throw new Exception( $"Expected two bounds, Got {range.Length}!" ); }
            ranges[ field ] = range;
        }
        throw new Exception( $"Key {field} DNE for this problem!" );
    }





}

}