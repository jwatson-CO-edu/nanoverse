//  CSCIx229 library
//  Willem A. (Vlakkies) Schreuder
#include <ctype.h>
#include "toolbox.hpp"

//
//  Load texture from BMP file
//

//
//  Reverse n bytes
//
static void Reverse(void* x,const int n){
   char* ch = (char*)x;
   for (int k=0;k<n/2;k++)
   {
      char tmp = ch[k];
      ch[k] = ch[n-1-k];
      ch[n-1-k] = tmp;
   }
}

//
//  Load texture from BMP file
//
unsigned int LoadTexBMP(const char* file){
   //  Open file
   FILE* f = fopen(file,"rb");
   if (!f) Fatal("Cannot open file %s\n",file);
   //  Check image magic
   unsigned short magic;
   if (fread(&magic,2,1,f)!=1) Fatal("Cannot read magic from %s\n",file);
   if (magic!=0x4D42 && magic!=0x424D) Fatal("Image magic not BMP in %s\n",file);
   //  Read header
   unsigned int dx,dy,off,k; // Image dimensions, offset and compression
   unsigned short nbp,bpp;   // Planes and bits per pixel
   if (fseek(f,8,SEEK_CUR) || fread(&off,4,1,f)!=1 ||
       fseek(f,4,SEEK_CUR) || fread(&dx,4,1,f)!=1 || fread(&dy,4,1,f)!=1 ||
       fread(&nbp,2,1,f)!=1 || fread(&bpp,2,1,f)!=1 || fread(&k,4,1,f)!=1)
     Fatal("Cannot read header from %s\n",file);
   //  Reverse bytes on big endian hardware (detected by backwards magic)
   if (magic==0x424D)
   {
      Reverse(&off,4);
      Reverse(&dx,4);
      Reverse(&dy,4);
      Reverse(&nbp,2);
      Reverse(&bpp,2);
      Reverse(&k,4);
   }
   //  Check image parameters
   unsigned int max;
   glGetIntegerv(GL_MAX_TEXTURE_SIZE,(int*)&max);
   if (dx<1 || dx>max) Fatal("%s image width %d out of range 1-%d\n",file,dx,max);
   if (dy<1 || dy>max) Fatal("%s image height %d out of range 1-%d\n",file,dy,max);
   if (nbp!=1)  Fatal("%s bit planes is not 1: %d\n",file,nbp);
   if (bpp!=24) Fatal("%s bits per pixel is not 24: %d\n",file,bpp);
   if (k!=0)    Fatal("%s compressed files not supported\n",file);
#ifndef GL_VERSION_2_0
   //  OpenGL 2.0 lifts the restriction that texture size must be a power of two
   for (k=1;k<dx;k*=2);
   if (k!=dx) Fatal("%s image width not a power of two: %d\n",file,dx);
   for (k=1;k<dy;k*=2);
   if (k!=dy) Fatal("%s image height not a power of two: %d\n",file,dy);
#endif

   //  Allocate image memory
   unsigned int size = 3*dx*dy;
   unsigned char* image = (unsigned char*) malloc(size);
   if (!image) Fatal("Cannot allocate %d bytes of memory for image %s\n",size,file);
   //  Seek to and read image
   if (fseek(f,off,SEEK_SET) || fread(image,size,1,f)!=1) Fatal("Error reading data from image %s\n",file);
   fclose(f);
   //  Reverse colors (BGR -> RGB)
   for (k=0;k<size;k+=3)
   {
      unsigned char temp = image[k];
      image[k]   = image[k+2];
      image[k+2] = temp;
   }

   //  Sanity check
   ErrCheck("LoadTexBMP");
   //  Generate 2D texture
   unsigned int texture;
   glGenTextures(1,&texture);
   glBindTexture(GL_TEXTURE_2D,texture);
   //  Copy image
   glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,dx,dy,0,GL_RGB,GL_UNSIGNED_BYTE,image);
   if (glGetError()) Fatal("Error in glTexImage2D %s %dx%d\n",file,dx,dy);
   //  Scale linearly when image size doesn't match
   glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
   glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);

   //  Free image memory
   free(image);
   //  Return texture name
   return texture;
}


//  Load an OBJ file
//  Vertex, Normal and Texture coordinates are supported
//  Materials are supported
//  Textures must be BMP files
//  Surfaces are not supported
//
//  WARNING:  This is a minimalist implementation of the OBJ file loader.  It
//  will only correctly load a small subset of possible OBJ files.  It is
//  intended to be a starting point to allow you to load models, but in order
//  for you to do this you will have to understand the OBJ format and most
//  likely will have to significantly enhance this code.
//
//  WARNING:  There are lots of really broken OBJ files on the internet.  Some
//  files may have correct surfaces, but the normals are complete junk and so
//  the lighting is totally broken.  So beware of which OBJ files you use.

//  Material structure
typedef struct
{
   char* name;                 //  Material name
   float Ka[4],Kd[4],Ks[4],Ns; //  Colors and shininess
   float d;                    //  Transparency
   int map;                    //  Texture
} mtl_t;

//  Material count and array
static int Nmtl=0;
static mtl_t* mtl=NULL;

//
//  Return true if CR or LF
//
static int CRLF(char ch)
{
   return ch == '\r' || ch == '\n';
}

//
//  Read line from file
//    Returns pointer to line or NULL on EOF
//
static int linelen=0;    //  Length of line
static char* line=NULL;  //  Internal storage for line
static char* readline(FILE* f){
   char ch;  //  Character read
   int k=0;  //  Character count
   while ((ch = fgetc(f)) != EOF)
   {
      //  Allocate more memory for long strings
      if (k>=linelen)
      {
         linelen += 8192;
         line = (char*)realloc(line,linelen);
         if (!line) Fatal("Out of memory in readline\n");
      }
      //  End of Line
      if (CRLF(ch))
      {
         // Eat extra CR or LF characters (if any)
         while ((ch = fgetc(f)) != EOF)
           if (!CRLF(ch)) break;
         //  Stick back the overrun
         if (ch != EOF) ungetc(ch,f);
         //  Bail
         break;
      }
      //  Pad character to line
      else
         line[k++] = ch;
   }
   //  Terminate line if anything was read
   if (k>0) line[k] = 0;
   //  Return pointer to line or NULL on EOF
   return k>0 ? line : NULL;
}

//
//  Read to next non-whitespace word
//  Note that this destroys line in the process
//
static char* getword(char** line){
   //  Skip leading whitespace
   while (**line && isspace(**line))
      (*line)++;
   if (!**line) return NULL;
   //  Start of word
   char* word = *line;
   //  Read until next whitespace
   while (**line && !isspace(**line))
      (*line)++;
   //  Mark end of word if not NULL
   if (**line)
   {
      **line = 0;
      (*line)++;
   }
   return word;
}

//
//  Read n floats
//
static void readfloat(char* line,int n,float x[]){
   for (int i=0;i<n;i++)
   {
      char* str = getword(&line);
      if (!str)  Fatal("Premature EOL reading %d floats\n",n);
      if (sscanf(str,"%f",x+i)!=1) Fatal("Error reading float %d\n",i);
   }
}

//
//  Read coordinates
//    n is how many coordiantes to read
//    N is the coordinate index
//    M is the number of coordinates
//    x is the array
//    This function adds more memory as needed in 8192 work chunks
//
static void readcoord(char* line,int n,float* x[],int* N,int* M){
   //  Allocate memory if necessary
   if (*N+n > *M)
   {
      *M += 8192;
      *x = (float*)realloc(*x,(*M)*sizeof(float));
      if (!*x) Fatal("Cannot allocate memory\n");
   }
   //  Read n coordinates
   readfloat(line,n,(*x)+*N);
   (*N)+=n;
}

//
//  Read string conditionally
//     Line must start with skip string
//     After skip sting return first word
//     getword terminates the line
//
static char* readstr(char* line,const char* skip){
   //  Check for a match on the skip string
   while (*skip && *line && *skip==*line)
   {
      skip++;
      line++;
   }
   //  Skip must be NULL for a match
   if (*skip || !isspace(*line)) return NULL;
   //  Read string
   return getword(&line);
}

//
//  Load materials from file
//
static void LoadMaterial(const char* file){
   int k=-1;
   char* line;
   char* str;

   //  Open file or return with warning on error
   FILE* f = fopen(file,"r");
   if (!f)
   {
      fprintf(stderr,"Cannot open material file %s\n",file);
      return;
   }

   //  Read lines
   while ((line = readline(f)))
   {
      //  New material
      if ((str = readstr(line,"newmtl")))
      {
         int l = strlen(str);
         //  Allocate memory for structure
         k = Nmtl++;
         mtl = (mtl_t*)realloc(mtl,Nmtl*sizeof(mtl_t));
         //  Store name
         mtl[k].name = (char*)malloc(l+1);
         if (!mtl[k].name) Fatal("Cannot allocate %d for name\n",l+1);
         strcpy(mtl[k].name,str);
         //  Initialize materials
         mtl[k].Ka[0] = mtl[k].Ka[1] = mtl[k].Ka[2] = 0;   mtl[k].Ka[3] = 1;
         mtl[k].Kd[0] = mtl[k].Kd[1] = mtl[k].Kd[2] = 0;   mtl[k].Kd[3] = 1;
         mtl[k].Ks[0] = mtl[k].Ks[1] = mtl[k].Ks[2] = 0;   mtl[k].Ks[3] = 1;
         mtl[k].Ns  = 0;
         mtl[k].d   = 0;
         mtl[k].map = 0;
      }
      //  If no material short circuit here
      else if (k<0)
      {}
      //  Ambient color
      else if (line[0]=='K' && line[1]=='a')
         readfloat(line+2,3,mtl[k].Ka);
      //  Diffuse color
      else if (line[0]=='K' && line[1] == 'd')
         readfloat(line+2,3,mtl[k].Kd);
      //  Specular color
      else if (line[0]=='K' && line[1] == 's')
         readfloat(line+2,3,mtl[k].Ks);
      //  Material Shininess
      else if (line[0]=='N' && line[1]=='s')
      {
         readfloat(line+2,1,&mtl[k].Ns);
         //  Limit to 128 for OpenGL
         if (mtl[k].Ns>128) mtl[k].Ns = 128;
      }
      //  Textures (must be BMP - will fail if not)
      else if ((str = readstr(line,"map_Kd")))
         mtl[k].map = LoadTexBMP(str);
      //  Ignore line if we get here
   }
   fclose(f);
}

//
//  Set material
//
static void SetMaterial(const char* name){
   //  Search materials for a matching name
   for (int k=0;k<Nmtl;k++)
      if (!strcmp(mtl[k].name,name))
      {
         //  Set material colors
         glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT  ,mtl[k].Ka);
         glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE  ,mtl[k].Kd);
         glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR ,mtl[k].Ks);
         glMaterialfv(GL_FRONT_AND_BACK,GL_SHININESS,&mtl[k].Ns);
         //  Bind texture if specified
         if (mtl[k].map)
         {
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D,mtl[k].map);
         }
         else
            glDisable(GL_TEXTURE_2D);
         return;
      }
   //  No matches
   fprintf(stderr,"Unknown material %s\n",name);
}

//
//  Load OBJ file
//
int LoadOBJ(const char* file){
   int  Nv,Nn,Nt;  //  Number of vertex, normal and textures
   int  Mv,Mn,Mt;  //  Maximum vertex, normal and textures
   float* V;       //  Array of vertexes
   float* N;       //  Array of normals
   float* T;       //  Array if textures coordinates
   char*  line;    //  Line pointer
   char*  str;     //  String pointer

   //  Open file
   FILE* f = fopen(file,"r");
   if (!f) Fatal("Cannot open file %s\n",file);

   // Reset materials
   mtl = NULL;
   Nmtl = 0;

   //  Start new displaylist
   int list = glGenLists(1);
   glNewList(list,GL_COMPILE);
   //  Push attributes for textures
   glPushAttrib(GL_ENABLE_BIT|GL_TEXTURE_BIT);

   //  Read vertexes and facets
   V  = N  = T  = NULL;
   Nv = Nn = Nt = 0;
   Mv = Mn = Mt = 0;
   while ((line = readline(f)))
   {
      //  Vertex coordinates (always 3)
      if (line[0]=='v' && line[1]==' ')
         readcoord(line+2,3,&V,&Nv,&Mv);
      //  Normal coordinates (always 3)
      else if (line[0]=='v' && line[1] == 'n')
         readcoord(line+2,3,&N,&Nn,&Mn);
      //  Texture coordinates (always 2)
      else if (line[0]=='v' && line[1] == 't')
         readcoord(line+2,2,&T,&Nt,&Mt);
      //  Read and draw facets
      else if (line[0]=='f')
      {
         line++;
         //  Read Vertex/Texture/Normal triplets
         glBegin(GL_POLYGON);
         while ((str = getword(&line)))
         {
            int Kv,Kt,Kn;
            //  Try Vertex/Texture/Normal triplet
            if (sscanf(str,"%d/%d/%d",&Kv,&Kt,&Kn)==3)
            {
               if (Kv<0 || Kv>Nv/3) Fatal("Vertex %d out of range 1-%d\n",Kv,Nv/3);
               if (Kn<0 || Kn>Nn/3) Fatal("Normal %d out of range 1-%d\n",Kn,Nn/3);
               if (Kt<0 || Kt>Nt/2) Fatal("Texture %d out of range 1-%d\n",Kt,Nt/2);
            }
            //  Try Vertex//Normal pairs
            else if (sscanf(str,"%d//%d",&Kv,&Kn)==2)
            {
               if (Kv<0 || Kv>Nv/3) Fatal("Vertex %d out of range 1-%d\n",Kv,Nv/3);
               if (Kn<0 || Kn>Nn/3) Fatal("Normal %d out of range 1-%d\n",Kn,Nn/3);
               Kt = 0;
            }
            //  Try Vertex index
            else if (sscanf(str,"%d",&Kv)==1)
            {
               if (Kv<0 || Kv>Nv/3) Fatal("Vertex %d out of range 1-%d\n",Kv,Nv/3);
               Kn = 0;
               Kt = 0;
            }
            //  This is an error
            else
               Fatal("Invalid facet %s\n",str);
            //  Draw vectors
            if (Kt) glTexCoord2fv(T+2*(Kt-1));
            if (Kn) glNormal3fv(N+3*(Kn-1));
            if (Kv) glVertex3fv(V+3*(Kv-1));
         }
         glEnd();
      }
      //  Use material
      else if ((str = readstr(line,"usemtl")))
         SetMaterial(str);
      //  Load materials
      else if ((str = readstr(line,"mtllib")))
         LoadMaterial(str);
      //  Skip this line
   }
   fclose(f);
   //  Pop attributes (textures)
   glPopAttrib();
   glEndList();

   //  Free materials
   for (int k=0;k<Nmtl;k++)
      free(mtl[k].name);
   free(mtl);

   //  Free arrays
   free(V);
   free(T);
   free(N);

   return list;
}
