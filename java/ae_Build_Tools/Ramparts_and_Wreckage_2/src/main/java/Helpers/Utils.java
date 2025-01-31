/*Utils.java
* James Watson, 105 754 866
* Provides functionality needed by multiple files, mostly Python stuff */

package Helpers;

public class Utils {

    public static int randrange( int lo , int hi ){
        // Just like Python!
        return lo + (int)( ( hi - lo ) * Math.random() );
    }

    public static int[] vec_add( int[] op1 , int[] op2 ){
        // Return a vector that is the element-wise sum of `op1` and `op2`
        int[] rtnArr = op1.clone();
        for( int i = 0 ; i < op1.length ; i++ ){
            rtnArr[i] += op2[i];
        }
        return rtnArr.clone();
    }

    public static int clamp( int val , int lo , int hi ){
        // Constrain `val` to the interval [ `lo` , `hi` ]
        return Math.min( Math.max( val , lo ) , hi );
    }

    public static boolean coinflip_P( float P ){
        // A weighted coin that returns True with probability `P`
        return Math.random() <= P;
    }

}
