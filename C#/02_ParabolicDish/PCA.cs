using OpenTK.Mathematics;
using MathNet.Numerics.LinearAlgebra;


namespace geo3d {

/// <summary>
/// Principal Component Analysis in 3D,
/// SLOP: https://claude.ai/share/79b43aee-2f43-4161-bd11-2bbb725ed2db
/// </summary>
public static class PCA {

    /// <summary>
    /// Compute Principal Axes for a collection of 3D points
    /// </summary>
    public static Vector3[] ComputePrincipalAxes( List<Vector3> points ){
        if (points == null || points.Count < 2){  throw new ArgumentException("Need at least 2 points for PCA.");  }

        // 1. Points -> n x 3 matrix (uses the ToMathNet() helper from before)
        Matrix<double> pointMatrix = points.ToMathNet();

        // 2. Mean-center each column (X, Y, Z)
        Vector<double> mean     = pointMatrix.ColumnSums() / points.Count;
        Matrix<double> centered = pointMatrix.Clone();
        for (int i = 0; i < centered.RowCount; i++){  centered.SetRow( i, centered.Row(i) - mean );  }

        // 3. Covariance matrix: (centered^T * centered) / (n - 1)
        Matrix<double> covariance = centered.TransposeThisAndMultiply( centered ) / ( points.Count - 1 );

        // 4. Eigendecomposition (covariance is symmetric, so eigenvalues are real)
        var /*------*/ evd /*----*/ = covariance.Evd();
        double[] /*-*/ eigenvalues  = [.. evd.EigenValues.Select(c => c.Real)];
        Matrix<double> eigenvectors = evd.EigenVectors;

        // 5. Evd() does NOT guarantee sorted order — sort indices descending by eigenvalue
        int[] order = [0, 1, 2];
        Array.Sort( order, (a, b) => eigenvalues[b].CompareTo( eigenvalues[a] ) );

        // 6. Pull out columns in sorted order, convert back to Vector3
        Vector3[] axes = new Vector3[3];
        for( int k = 0; k < 3; k++ ){  axes[k] = eigenvectors.Column( order[k] ).ToOpenTK();  }

        return axes;
    }
}


public static class Ops2D3D {

    
    public static List<Vector2> Project3dPointsTo2d( List<Vector3> points ){
        List<Vector2> rtnLst = [];
        Vector3 /*-*/ center = MathVec3.UniformPointCentroid( points );
        Vector3[]     axes   = PCA.ComputePrincipalAxes( points );
        Vector3 /*-*/ xAxis  = axes[0].Normalized();
        Vector3 /*-*/ yAxis  = axes[1].Normalized();
        Vector3 /*-*/ offsetPnt;
        rtnLst.Capacity = points.Count;

        foreach( Vector3 point in points ){
            offsetPnt = point - center;
            rtnLst.Add( new Vector2( Vector3.Dot( offsetPnt, xAxis ), Vector3.Dot( offsetPnt, yAxis ) ) );
        }

        return rtnLst;
    }
}

}