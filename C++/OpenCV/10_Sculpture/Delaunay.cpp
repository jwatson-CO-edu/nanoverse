/*
An Algorithm for Interpolating Irregularly-Spaced Data
with Applications in Terrain Modelling
Written by Paul Bourke
Presented at Pan Pacific Computer Conference, Beijing, China.
January 1989
*/

#include "Delaunay.hpp"

using namespace std; 

////////////////////////////////////////////////////////////////////////
// CircumCircle() :
//   Return true if a point (xp,yp) is inside the circumcircle made up
//   of the points (x1,y1), (x2,y2), (x3,y3)
//   The circumcircle centre is returned in (xc,yc) and the radius r
//   Note : A point on the edge is inside the circumcircle
////////////////////////////////////////////////////////////////////////

int CircumCircle(double xp, double yp, double x1, double y1, double x2, 
double y2, double x3, double y3, double &xc, double &yc, double &r){
  double m1, m2, mx1, mx2, my1, my2;
  double dx, dy, rsqr, drsqr;

/* Check for coincident points */
if(abs(y1 - y2) < EPSILON && abs(y2 - y3) < EPSILON)
  return(false);
if(abs(y2-y1) < EPSILON){ 
  m2 = - (x3 - x2) / (y3 - y2);
  mx2 = (x2 + x3) / 2.0;
  my2 = (y2 + y3) / 2.0;
  xc = (x2 + x1) / 2.0;
  yc = m2 * (xc - mx2) + my2;
}else if(abs(y3 - y2) < EPSILON){ 
        m1 = - (x2 - x1) / (y2 - y1);
        mx1 = (x1 + x2) / 2.0;
        my1 = (y1 + y2) / 2.0;
        xc = (x3 + x2) / 2.0;
        yc = m1 * (xc - mx1) + my1;
      }else{
         m1 = - (x2 - x1) / (y2 - y1); 
         m2 = - (x3 - x2) / (y3 - y2); 
         mx1 = (x1 + x2) / 2.0; 
         mx2 = (x2 + x3) / 2.0;
         my1 = (y1 + y2) / 2.0;
         my2 = (y2 + y3) / 2.0;
         xc = (m1 * mx1 - m2 * mx2 + my2 - my1) / (m1 - m2); 
         yc = m1 * (xc - mx1) + my1; 
       }
  dx = x2 - xc;
  dy = y2 - yc;
  rsqr = dx * dx + dy * dy;
  r = sqrt(rsqr); 
  dx = xp - xc;
  dy = yp - yc;
  drsqr = dx * dx + dy * dy;
  return((drsqr <= rsqr) ? true : false);
}
///////////////////////////////////////////////////////////////////////////////
// Triangulate() :
//   Triangulation subroutine
//   Takes as input NV vertices in array pxyz
//   Returned is a list of ntri triangular faces in the array v
//   These triangles are arranged in a consistent clockwise order.
//   The triangle array 'v' should be malloced to 3 * nv
//   The vertex array pxyz must be big enough to hold 3 more points
//   The vertex array must be sorted in increasing x values say: qsort(p,nv,sizeof(XYZ),XYZCompare);
//
//   Inputs:
//   int nv , ___ Number of vertices
//   XYZ pxyz[] , Array of vertices
// 
//   Outputs:
//   ITRIANGLE v[] , Array of triangles
//   int &ntri , ___ Number of triangles found
///////////////////////////////////////////////////////////////////////////////

int Triangulate(int nv, XYZ pxyz[], ITRIANGLE v[], int &ntri){
  int *complete = NULL;
  IEDGE *edges = NULL; 
  IEDGE *p_EdgeTemp;
  int nedge = 0;
  int trimax, emax = 200;
//   int status = 0;
  int inside;
  int i, j, k;
  double xp, yp, x1, y1, x2, y2, x3, y3, xc = 0 , yc, r = 0;
  double xmin, xmax, ymin, ymax, xmid, ymid;
  double dx, dy, dmax; 

/* Allocate memory for the completeness list, flag for each triangle */
  trimax = 4 * nv;
  complete = new int[trimax];
/* Allocate memory for the edge list */
  edges = new IEDGE[emax];
/*
      Find the maximum and minimum vertex bounds.
      This is to allow calculation of the bounding triangle
*/
  xmin = pxyz[0].x;
  ymin = pxyz[0].y;
  xmax = xmin;
  ymax = ymin;
  for(i = 1; i < nv; i++){
    if (pxyz[i].x < xmin) xmin = pxyz[i].x;
    if (pxyz[i].x > xmax) xmax = pxyz[i].x;
    if (pxyz[i].y < ymin) ymin = pxyz[i].y;
    if (pxyz[i].y > ymax) ymax = pxyz[i].y;
  }
  dx = xmax - xmin;
  dy = ymax - ymin;
  dmax = (dx > dy) ? dx : dy;
  xmid = (xmax + xmin) / 2.0;
  ymid = (ymax + ymin) / 2.0;
/*
   Set up the supertriangle
   his is a triangle which encompasses all the sample points.
   The supertriangle coordinates are added to the end of the
   vertex list. The supertriangle is the first triangle in
   the triangle list.
*/
  pxyz[nv+0].x = xmid - 20 * dmax;
  pxyz[nv+0].y = ymid - dmax;
  pxyz[nv+1].x = xmid;
  pxyz[nv+1].y = ymid + 20 * dmax;
  pxyz[nv+2].x = xmid + 20 * dmax;
  pxyz[nv+2].y = ymid - dmax;
  v[0].p1 = nv;
  v[0].p2 = nv+1;
  v[0].p3 = nv+2;
  complete[0] = false;
  ntri = 1;
/*
   Include each point one at a time into the existing mesh
*/
  for(i = 0; i < nv; i++){
    xp = pxyz[i].x;
    yp = pxyz[i].y;
    nedge = 0;
/*
     Set up the edge buffer.
     If the point (xp,yp) lies inside the circumcircle then the
     three edges of that triangle are added to the edge buffer
     and that triangle is removed.
*/
  for(j = 0; j < ntri; j++){
    if(complete[j])
      continue;
    x1 = pxyz[v[j].p1].x;
    y1 = pxyz[v[j].p1].y;
    x2 = pxyz[v[j].p2].x;
    y2 = pxyz[v[j].p2].y;
    x3 = pxyz[v[j].p3].x;
    y3 = pxyz[v[j].p3].y;
    inside = CircumCircle(xp, yp, x1, y1, x2, y2, x3, y3, xc, yc, r); // This will set 'xc' and 'r' 
    if (xc + r < xp)
// Suggested
// if (xc + r + EPSILON < xp)
      complete[j] = true;
    if(inside){
/* Check that we haven't exceeded the edge list size */
      if(nedge + 3 >= emax){
        emax += 100;
        p_EdgeTemp = new IEDGE[emax];
        for (int i = 0; i < nedge; i++) { // Fix by John Bowman
          p_EdgeTemp[i] = edges[i];   
        }
        delete []edges;
        edges = p_EdgeTemp;
      }
      edges[nedge+0].p1 = v[j].p1;
      edges[nedge+0].p2 = v[j].p2;
      edges[nedge+1].p1 = v[j].p2;
      edges[nedge+1].p2 = v[j].p3;
      edges[nedge+2].p1 = v[j].p3;
      edges[nedge+2].p2 = v[j].p1;
      nedge += 3;
      v[j] = v[ntri-1];
      complete[j] = complete[ntri-1];
      ntri--;
      j--;
    }
  }
/*
  Tag multiple edges
  Note: if all triangles are specified anticlockwise then all
  interior edges are opposite pointing in direction.
*/
  for(j = 0; j < nedge - 1; j++){
    for(k = j + 1; k < nedge; k++){
      if((edges[j].p1 == edges[k].p2) && (edges[j].p2 == edges[k].p1)){
        edges[j].p1 = -1;
        edges[j].p2 = -1;
        edges[k].p1 = -1;
        edges[k].p2 = -1;
      }
       /* Shouldn't need the following, see note above */
      if((edges[j].p1 == edges[k].p1) && (edges[j].p2 == edges[k].p2)){
        edges[j].p1 = -1;
        edges[j].p2 = -1;
        edges[k].p1 = -1;
        edges[k].p2 = -1;
      }
    }
  }
/*
     Form new triangles for the current point
     Skipping over any tagged edges.
     All edges are arranged in clockwise order.
*/
  for(j = 0; j < nedge; j++) {
    if(edges[j].p1 < 0 || edges[j].p2 < 0)
      continue;
    v[ntri].p1 = edges[j].p1;
    v[ntri].p2 = edges[j].p2;
    v[ntri].p3 = i;
    complete[ntri] = false;
    ntri++;
  }
}
/*
      Remove triangles with supertriangle vertices
      These are triangles which have a vertex number greater than nv
*/
  for(i = 0; i < ntri; i++) {
    if(v[i].p1 >= nv || v[i].p2 >= nv || v[i].p3 >= nv) {
      v[i] = v[ntri-1];
      ntri--;
      i--;
    }
  }
  delete[] edges;
  delete[] complete;
  return 0;
} 

void randomize(){
  srand((time_t) time(NULL));  
}

int random(int n){
  return rand()%n; 
}

int XYZCompare(const void *v1, const void *v2){
  XYZ *p1, *p2;
    
  p1 = (XYZ*)v1;
  p2 = (XYZ*)v2;
  if(p1->x < p2->x)
    return(-1);
  else if(p1->x > p2->x)
         return(1);
       else
         return(0);
}

void outputtriangle(int &nv, XYZ p[], ITRIANGLE v[], int &ntri){
//   int X, Y, i = 0;
//   int max = 10;
//   double x, y;

  for(int i = 0; i < ntri; i++){// replace cout by a compatible lineto to trace
    cout<<(int)p[v[i].p1].x<<" "<< (int)p[v[i].p1].y<<" "<< (int)p[v[i].p2].x
    <<" "<< (int)p[v[i].p2].y<<"\n";
    cout<<(int)p[v[i].p2].x<<" "<< (int)p[v[i].p2].y<<" "<< (int)p[v[i].p3].x
    <<" "<< (int)p[v[i].p3].y<<"\n";
    cout<<(int)p[v[i].p3].x<<" "<< (int)p[v[i].p3].y<<" "<< (int)p[v[i].p1].x
    <<" "<< (int)p[v[i].p1].y<<"\n";
  }
}

/// ############## MAIN ###################################################################################################################

#ifdef DELAUNAY_MAIN

int main(){
  ITRIANGLE *v = NULL;
  int max = 10;
  XYZ *p = new XYZ[max]; 
  XYZ *p_Temp = NULL;   
  int nv = 0;
  int X, Y;
  int i;
  int ntri = 0;
  double x, y, z;
  bool b_Ok = false;
    
  randomize();
  nv = 0;
  p = new XYZ[max];
  while (nv != n_MaxPoints){
    do{
      b_Ok = true;
      x = (double)random(500);
      y = (double)random(500);
      for(int n_Cpt = 0; n_Cpt <= nv; n_Cpt++){
        if((x == p[n_Cpt].x) && (y == p[n_Cpt].y)) b_Ok = false;
      }// to avoid similar points in the array
    }while(!b_Ok);
    if (nv >= max){
      max = max * 2;            // double the size of the array if necessary
      p_Temp = new XYZ[max]; 
      for (int i = 0; i < nv; i++) {
        p_Temp[i] = p[i];  
      }
      delete []p;  
      p = p_Temp; 
    }   
    p[nv].x = x * 1.0;
    p[nv].y = y * 1.0;
    nv++;
  }
  p_Temp = new XYZ[nv + 3]; 
  for (int i = 0; i < nv; i++) {
    p_Temp[i] = p[i];      
  }
  delete []p;           
  p = p_Temp;
  v = new ITRIANGLE[3 * nv]; 
  qsort(p, nv, sizeof(XYZ), XYZCompare);
  Triangulate(nv, p, v, ntri);
  outputtriangle(nv, p, v, ntri); // use this fonction to trace the mesh (via OpenGL, DirectX, ...)
  delete []p;
  delete []v;
  p = NULL;
  v = NULL;
  system("pause"); // remove under Linux
  return 0;
}

#endif