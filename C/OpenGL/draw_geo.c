////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "geometry.h"



////////// GEOMETRY HELPERS ////////////////////////////////////////////////////////////////////////

void Vertex_sphr( float th , float ph ){
    // Draw vertex in polar coordinates
    // Author: Willem A. (Vlakkies) Schreüder  
    glVertex3d( Sin( th )*Cos( ph ), 
                Sin( ph ), 
                Cos( th )*Cos( ph ) );
}



////////// 3D GEOMETRY DRAWING FUNCTIONS ///////////////////////////////////////////////////////////

void draw_cuboid_lined( float x , float y , float z ,
                        float dx , float dy , float dz ,
                        float fillColor[3] , float lineColor[3] ){
    // Draw a cube at (x,y,z) dimensions (dx,dy,dz) 
    float lineOffset = 1.005;
    //  Save transformation
    glPushMatrix();
    glColor3f( fillColor[0] , fillColor[1] , fillColor[2] );
    //  Offset
    glTranslated( x , y , z );
    glScaled( dx/2.0f , dy/2.0f , dz/2.0f );
    
    //  Cube
    glBegin(GL_QUADS);
        //  Front
        glVertex3f(-1,-1, 1);
        glVertex3f(+1,-1, 1);
        glVertex3f(+1,+1, 1);
        glVertex3f(-1,+1, 1);
        //  Back
        glVertex3f(+1,-1,-1);
        glVertex3f(-1,-1,-1);
        glVertex3f(-1,+1,-1);
        glVertex3f(+1,+1,-1);
        //  Right
        glVertex3f(+1,-1,+1);
        glVertex3f(+1,-1,-1);
        glVertex3f(+1,+1,-1);
        glVertex3f(+1,+1,+1);
        //  Left
        glVertex3f(-1,-1,-1);
        glVertex3f(-1,-1,+1);
        glVertex3f(-1,+1,+1);
        glVertex3f(-1,+1,-1);
        //  Top
        glVertex3f(-1,+1,+1);
        glVertex3f(+1,+1,+1);
        glVertex3f(+1,+1,-1);
        glVertex3f(-1,+1,-1);
        //  Bottom
        glVertex3f(-1,-1,-1);
        glVertex3f(+1,-1,-1);
        glVertex3f(+1,-1,+1);
        glVertex3f(-1,-1,+1);
    //  End
    glEnd();
    
    // Draw outline
    float d = lineOffset;
    glColor3f( lineColor[0] , lineColor[1] , lineColor[2] );
    
    glBegin( GL_LINES );
        // Bottom
        glVertex3f(-d,-d,-d);
        glVertex3f(+d,-d,-d);
        
        glVertex3f(+d,-d,-d);
        glVertex3f(+d,+d,-d);
        
        glVertex3f(+d,+d,-d);
        glVertex3f(-d,+d,-d);
        
        glVertex3f(-d,+d,-d);
        glVertex3f(-d,-d,-d);

        // Top
        glVertex3f(-d,-d,+d);
        glVertex3f(+d,-d,+d);
        
        glVertex3f(+d,-d,+d);
        glVertex3f(+d,+d,+d);
        
        glVertex3f(+d,+d,+d);
        glVertex3f(-d,+d,+d);
        
        glVertex3f(-d,+d,+d);
        glVertex3f(-d,-d,+d);
        
        // Sides
        glVertex3f(-d,-d,+d);
        glVertex3f(-d,-d,-d);
        
        glVertex3f(+d,-d,+d);
        glVertex3f(+d,-d,-d);
        
        glVertex3f(+d,+d,+d);
        glVertex3f(+d,+d,-d);
        
        glVertex3f(-d,+d,+d);
        glVertex3f(-d,+d,-d);
        
    glEnd();
    //  Undo transformations
    glPopMatrix();
}


void draw_sphere( vec4f center, float radius, vec4f color ){
    // Draw a sphere (version 2) at (x,y,z) radius (r)
    // Author: Willem A. (Vlakkies) Schreüder  
    const int d = 5;
    int /*-*/ th , ph;
    float     x = center.x;
    float     y = center.y;
    float     z = center.z;
    float     r = radius;

    //  Save transformation
    glPushMatrix();
    //  Offset and scale
    glTranslated( x , y , z );
    glScaled( r , r , r );

    glColor3f( color.r , color.g , color.b );

    //  Latitude bands
    for( ph = -90 ; ph < 90 ; ph += d ){
        glBegin( GL_QUAD_STRIP );
        for( th = 0 ; th <= 360 ; th += d ){
            Vertex_sphr( th , ph     );
            Vertex_sphr( th , ph + d );
        }
        glEnd();
    }

    //  Undo transformations
    glPopMatrix();
}


void draw_grid_org_XY( float gridSize, uint xPlusMinus, uint yPlusMinus, 
                       float lineThic, vec4f color ){
    // Draw a square grid centered at the origin, extending 'xPlusMinus' units in X and 'yPlusMinus' units in Y
    
    float xMin = - gridSize * xPlusMinus, 
          xMax =   gridSize * xPlusMinus,
          yMin = - gridSize * yPlusMinus, 
          yMax =   gridSize * yPlusMinus,
          zOfs = 0.005;
    
    glLineWidth( lineThic );
    glClr4f( color );
    
    glBegin( GL_LINES );
    
        // 1. Draw the axis , X
        glVertex3d( 0 , yMin , 0 ); // Bgn
        glVertex3d( 0 , yMax , 0 ); // End
        // 2. Draw the axis , Y
        glVertex3d( xMin , 0 , 0 ); // Bgn
        glVertex3d( xMax , 0 , 0 ); // End
        
        // 3. Draw the grid , X
        for( uint i = 0 ; i < xPlusMinus ; i++ ){
            // Plus
            glVertex3d(  gridSize * i , yMin , zOfs ); // Bgn
            glVertex3d(  gridSize * i , yMax , zOfs ); // End
            // Minus
            glVertex3d( -gridSize * i , yMin , zOfs ); // Bgn
            glVertex3d( -gridSize * i , yMax , zOfs ); // End
        }
        
        // 3. Draw the grid , Y
        for( uint i = 0 ; i < yPlusMinus ; i++ ){
            // Plus
            glVertex3d( xMin ,  gridSize * i , zOfs ); // Bgn
            glVertex3d( xMax ,  gridSize * i , zOfs ); // End
            // Minus
            glVertex3d( xMin , -gridSize * i , zOfs ); // Bgn
            glVertex3d( xMax , -gridSize * i , zOfs ); // End
        }
        
    glEnd();
}
