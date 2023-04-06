/***********************************************************************
 * 2020/11                                                             *
 *                                                                     *
 * Wavefront OBJ model loader implemented from scratch by Codifies     *
 * seperates mesh into a mesh per material                             *
 *                                                                     *
 * no header just use                                                  *
 *                                                                     *
 *     extern Model LoadObj(const char* filename);                     *
 *                                                                     *
 * TODO's                                                              *
 *                                                                     *
 * only diffuse map and colour used                                    *
 *                                                                     *
 ***********************************************************************/

/*
 * Copyright (c) 2020 Chris Camacho (Codifies -  http://bedroomcoders.co.uk/)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */ 

#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#if defined(_WIN32)
    #include <direct.h>     // Required for: _chdir() [Used in LoadObj()]
    #define CHDIR _chdir
#else
    #include <unistd.h>     // Required for: chdir() (POSIX) [Used in LoadObj()]
    #define CHDIR chdir
#endif

#include "rlgl.h"
#ifndef DEFAULT_MESH_VERTEX_BUFFERS
    #define DEFAULT_MESH_VERTEX_BUFFERS    7    // Number of vertex buffers (VBO) per mesh
#endif


// structure to hold face indexes
// initially before triangulation a face can have > 3 verts
typedef struct faceInfo {
    int *vi, *ti, *ni;
    int numVerts;
    struct faceInfo* next; // a face can only be in a single list....
} faceInfo;

// a list of faces
typedef struct faceList {
    faceInfo* first;
    faceInfo* last;
    int faceCount;
} faceList;

// TODO make this set by material count
#define MAXFACEMATS 128

// a list of all faces by material
faceList allFaces[MAXFACEMATS] = { 0 };


// copy string till the end of line in source string
static inline void strCpyEOL(char* src, char* dst) 
{
    int i=0;
    char tmp[1024];
    while (src[0]!='\n' && src[0]!='\r' && src[0]!=0) {
        tmp[i++] = src[0];
        src++;
    }
    tmp[i]=0;
    strncpy(dst, tmp, 1024);
}

// scan forward for the start of the next line or leave
// on NULL char at end...
static inline char* nextLine(char* line) 
{
    while (*line != '\n') { line++; } // find end of line
    while (*line != 0 && *line < 32) { line++; } // find start of next
    return line;  
}


// maintain the face count and chain of next ptrs
static void addFace(faceInfo* f, int matID) 
{
    if (allFaces[matID].first==NULL) {  allFaces[matID].first = f;  }
    
    if (allFaces[matID].last!=NULL) { 
        allFaces[matID].last->next = f;
    }
        
    allFaces[matID].last = f;
    f->next = NULL;
    allFaces[matID].faceCount++;
}

// release all the memory allocated to the faces
static void releaseFaces()
{
    for (int i=0; i<MAXFACEMATS; i++) {
        faceInfo* node = allFaces[i].first;
        while (node) {
            faceInfo* nextNode = node->next;
            RL_FREE(node->vi);
            if (node->ti) RL_FREE(node->ti);
            if (node->ni) RL_FREE(node->ni);
            RL_FREE(node);
            node = nextNode;
        }
    }
}

// count how many times a character is in a string 
static inline int countChars(char* in, char c) 
{
    int i=0, n=0;
    while (in[i] != '\0') {
        if (in[i] == c) {  n++;  }
        i++;
    }
    return n;
}

/* a triangle face could be formatted like.... 
 * case 1   v/t/n
 * case 2   v//n 
 * case 3   v/t
 * case 4   v
 */
static void parseVert(char* in, int corner, faceInfo* ti) 
{
    int ns = countChars(in, '/');
    
    if (ns == 2) {                          // case 1 or 2
        if (!strstr(in,"//")) {             // case 1
            sscanf( in, "%i/%i/%i", &ti->vi[corner], &ti->ti[corner], &ti->ni[corner] );
        } else {                            // case 2
            sscanf( in, "%i//%i", &ti->vi[corner], &ti->ni[corner] );
        }
    }
    
    if (ns == 1) {                          // case 3
        sscanf( in, "%i/%i", &ti->vi[corner], &ti->ti[corner]);
    }
    
    if (ns == 0) {                          // case 4
        sscanf( in, "%i", &ti->vi[corner]);
    }  
}

// copy from one string to another till you hit the end of the string
// or a specific character in the source string
// returns the location in the source string 1 character after
// the token is found, so one string can be split into multiple.
static char* cpyStrTill(char* src, char* dst, char till) 
{
    int i=0;
    while (src[i] != till && src[i] != '\r' && src[i] != '\n' && src[i] != 0) {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
    i++;
    return &src[i];
} 

static faceInfo* parseFace(char* in) 
{
    // first split into 3 strings
    char s1[80],s2[80],s3[80];
    
    char* s = in;
    s = cpyStrTill( s, s1, ' ');
    s = cpyStrTill( s, s2, ' ');
    s = cpyStrTill( s, s3, ' ');
    
    char* ss = s; // save the point past first tri
    int ex = 0; // count the extra verts
    while(s[0]) {
        char l[80];
        char* ls = s;
        s = cpyStrTill(s, l, ' ');
        if (s!=ls+1) {  ex++;  }
    }
    
    // allocate space for triangle / polygon verts 
    faceInfo* face = RL_CALLOC(1, sizeof(faceInfo)); 
    face->numVerts = ex+3;
    face->vi = RL_CALLOC(ex+3, sizeof(int));
    face->ti = RL_CALLOC(ex+3, sizeof(int));
    face->ni = RL_CALLOC(ex+3, sizeof(int));
    
    
    // then parse each sub string pf 1st tri
    parseVert(s1, 0, face);
    parseVert(s2, 1, face);
    parseVert(s3, 2, face);
    
    s=ss; // if its a polygon add in the extra verts
    int c = 3;
    while(s[0]) {
        char l[80];
        char* ls = s;
        s = cpyStrTill(s, l, ' ');
        if (s!=ls+1) {  parseVert(l, c, face); c++; }
    }

    return face;
}



Model LoadObj(const char* filename)
{
    // codifies 2021 written by hand to load and keep seperate
    // the different sub meshes in an OBJ file
    char *line;
    Model model = { 0 };
    char currentDir[1024] = { 0 };
    char tmpStr[1024] = { 0 };
    
    // clear all the face lists
    for (int i=0; i< MAXFACEMATS; i++) {
        allFaces[i].faceCount = 0;
        allFaces[i].first = NULL;
        allFaces[i].last = NULL;
    }
    
    // save current dir to restore later
    strcpy(currentDir, GetWorkingDirectory());

    // load in the obj text file
    char* objdata = LoadFileText(filename);
    char* mtldata = NULL;
    CHDIR(GetDirectoryPath(filename));
    
    if (!objdata) {
        return model;
    }
    
    // scan through obj find the material file and also
    // total the vert, normals, texture coords and faces
    int vc = 0;        // counts for verts, norms, tx coords and faces
    int nc = 0;
    int tc = 0;
    int faces = 0;
    line = objdata;
    
    while(line[0]!='\0') 
    {
        // spec says there should only be one mtllib
        if (strncmp("mtllib ", line, 7)==0) {
            strCpyEOL(line+7, tmpStr);
            mtldata = LoadFileText(tmpStr);
        }
        // counts
        if (strncmp("v ", line, 2)==0) {  vc++;  }
        if (strncmp("vn ", line, 3)==0) {  nc++;  }
        if (strncmp("vt ", line, 3)==0) {  tc++;  }
        if (strncmp("f ", line, 2)==0) {  faces++;  }

        line = nextLine(line);
    }

    // count the materials
    int materialCount=0;
    line = mtldata;
    if (line) {  // there need not be a mtl file
        while (line[0] != '\0') {
            if (strncmp("newmtl ", line, 7)==0) {
                materialCount++;
            }
            line = nextLine(line); 
        }
    }
    
    // at least one mesh/material which if no mtl will be default WHITE
    if (materialCount==0)  {  materialCount=1;  }
    //printf("material count %i\n", materialCount);
    TraceLog(LOG_INFO, "LoadObj: material count %i", materialCount);
    
    char matNames[materialCount][1024];
    char txNames[materialCount][1024];
    Color colours[materialCount];
    // set the first colour to white in case we have no
    // materials...
    colours[0] = WHITE;

    // clear out the texture names strings
    for (int i=0; i<materialCount; i++) {
        txNames[i][0]='\0';
    }

    // get the textures and colours from the materials file
    line = mtldata;
    
    if (line) {  // is there a mtl file?
        int m=0;
        while(line[0]!='\0') 
        {
            // save each of the material names
            if (strncmp("newmtl ", line, 7)==0) {
                strCpyEOL(line+7, matNames[m]);
                m++;
            }
            
            // TODO implement the other map types here
            
            // diffuse colour
            if (strncmp("Kd ", line, 3)==0) {
                float r,g,b;
                sscanf( line+3, "%f %f %f", &r, &g, &b );
                colours[m-1].r = r * 255;
                colours[m-1].g = g * 255;
                colours[m-1].b = b * 255;
                colours[m-1].a = 255;
            }
            
            // diffuse map
            if (strncmp("map_Kd", line, 6)==0) {
                strCpyEOL(line+7, txNames[m-1]);
            }
      
            line = nextLine(line);
        }
    }
    
    line = objdata;
    int currentMat = 0;
    
    // temporary storage for the vertex information
    float verts[vc*3];
    float tx[tc*2];
    float norms[nc*3];
    
    // indexes
    int vi=0, ti=0, ni=0;

    // count up the verts for each material
    // get all the vert data etc ready to
    // split between materials
    while (line[0] != '\0') 
    {
        if (strncmp("usemtl ", line, 7)==0) {
            strCpyEOL(line+7, tmpStr);
            for (int i = 0; i < materialCount; i++) {
                // simple linear search (KISS principle)
                // usually only a handful of materials so not
                // worth the overhead of hashmap or other techniques
                if (strcmp(tmpStr, matNames[i])==0) {
                    currentMat = i;
                    break;
                }
            } 
        }
        
        // count and collect the faces
        // at this point some faces might have more that 3 verts
        if (strncmp("f ", line, 2)==0) {
            strCpyEOL(line+2, tmpStr);
            faceInfo *f = parseFace(tmpStr);
            addFace(f, currentMat);
        }
        
        // store all the vertices positions
        if (strncmp("v ", line, 2)==0) {
            float v0,v1,v2;
            sscanf( line+2, "%f %f %f", &v0, &v1, &v2 );
            
            verts[vi] = v0;
            verts[vi+1] = v1;
            verts[vi+2] = v2;
            vi+=3;
        }
        
        // texture coords
        if (strncmp("vt ", line, 3)==0) {
            float t0,t1;
            sscanf( line+3, "%f %f", &t0, &t1 );
            
            tx[ti] = t0;
            tx[ti+1] = t1;
            ti+=2;
        }
        
        // normals
        if (strncmp("vn ", line, 3)==0) {
            float v0,v1,v2;
            sscanf( line+3, "%f %f %f", &v0, &v1, &v2 );
            
            norms[ni] = v0;
            norms[ni+1] = v1;
            norms[ni+2] = v2;
            ni+=3;
        }   
             
        line = nextLine(line);
    }


    //if (triangulate) {
        int ntf = 0;
        // triangulation
        for (int i=0; i < materialCount; i++) {
            // get all faces for each material in turn
            faceInfo* fi = allFaces[i].first;
            while(fi) {
                faceInfo* nextNode = fi->next;
                // if its not a triangle
                // turn the poly into a triangle fan
                if (fi->numVerts > 3) {
                    // loop through extra points making new (tri) faces
                    // set this faces facecount to 3
                    for (int j = 2; j < fi->numVerts-1; j++) {

                        faceInfo* nf = RL_CALLOC(1, sizeof(faceInfo));
                        nf->vi = RL_CALLOC(3, sizeof(int));
                        nf->ti = RL_CALLOC(3, sizeof(int));
                        nf->ni = RL_CALLOC(3, sizeof(int));
                        
                        nf->vi[0] = fi->vi[0];  nf->ti[0] = fi->ti[0];  nf->ni[0] = fi->ni[0];
                        nf->vi[1] = fi->vi[j];  nf->ti[1] = fi->ti[j];  nf->ni[1] = fi->ni[j];
                        nf->vi[2] = fi->vi[j+1];  nf->ti[2] = fi->ti[j+1];  nf->ni[2] = fi->ni[j+1];

                        nf->numVerts = 3;
                        addFace(nf, i);
                        ntf++;
                    }
                    fi->numVerts = 3;
                }
                
                fi = nextNode;
            }        
        }

        TraceLog(LOG_INFO,"LoadObj: model triangulation added %i tri's",ntf);
    //}
    
    
    model.meshCount = materialCount;
    model.materialCount = materialCount;
    
    // running index counts for each material indexes
    int mvi[materialCount];
    int mti[materialCount];
    int mni[materialCount];
    
    model.meshMaterial = RL_CALLOC(materialCount, sizeof(int));
    model.materials = (Material *)RL_CALLOC(model.materialCount, sizeof(Material));
    model.meshes = RL_CALLOC(materialCount, sizeof(Mesh));

    
    // finally all the meta is gathered, build the meshes...
    
    int tfc = 0;
    // first set up the model materials
    for (int i=0; i < materialCount; i++) {
        TraceLog(LOG_INFO,"LoadObj: material %i %s face count %i",i,matNames[i],allFaces[i].faceCount);
        tfc += allFaces[i].faceCount;
        mvi[i]=0; mti[i]=0; mni[i]=0;

        model.meshes[i].vertexCount = allFaces[i].faceCount*3;
        model.meshes[i].triangleCount = allFaces[i].faceCount;

        model.meshes[i].vertices = RL_CALLOC(allFaces[i].faceCount*9, sizeof(float));
        model.meshes[i].normals = RL_CALLOC(allFaces[i].faceCount*9, sizeof(float));
        model.meshes[i].texcoords = RL_CALLOC(allFaces[i].faceCount*6, sizeof(float));

        model.meshMaterial[i] = i;
        
        // TODO more than just DIFFUSE maps....!
        model.materials[i] = LoadMaterialDefault();

        if (txNames[i][0]!='\0') {
            model.materials[i].maps[MAP_DIFFUSE].texture = LoadTexture(txNames[i]);
        }
        model.materials[i].maps[MAP_DIFFUSE].color = colours[i];
        model.materials[i].maps[MAP_DIFFUSE].value = 0.0f;
    }
    TraceLog(LOG_INFO,"LoadObj: total face count %i",tfc);
        

    // look up the vert data for the mesh buffers from the face indexes
    
    for (int i=0; i < materialCount; i++) {
        mvi[i] = 0; mti[i] = 0; mni[i] = 0;
    }        
    
    for (int i=0; i < materialCount; i++) {
        faceInfo* fi = allFaces[i].first;
        while(fi) {
            faceInfo* nextNode = fi->next;

            for (int p=0; p<3; p++) {
                int idx;
                // never seen it in the wild but according to spec
                // -tive indexes are relative to the end of the buffer
                if (fi->vi[p] < 0) {  fi->vi[p] = vc - fi->vi[p];  }
                idx = (fi->vi[p]-1)*3;
                model.meshes[i].vertices[mvi[i]] = verts[idx];
                model.meshes[i].vertices[mvi[i]+1] = verts[idx+1];
                model.meshes[i].vertices[mvi[i]+2] = verts[idx+2];
                mvi[i]+=3;
                
                if (fi->ti[p] < 0) {  fi->ti[p] = tc - fi->ti[p];  }
                if (fi->ti[p] != 0) {
                    idx = (fi->ti[p]-1)*2;
                    model.meshes[i].texcoords[mti[i]] = tx[idx];
                    model.meshes[i].texcoords[mti[i]+1] = -tx[idx+1];
                    mti[i]+=2;
                }
                
                if (fi->ni[p] < 0) {  fi->ni[p] = nc - fi->ni[p];  }
                if (fi->ni[p]) {
                    idx = (fi->ni[p]-1)*3;
                    model.meshes[i].normals[mni[i]] = norms[idx];
                    model.meshes[i].normals[mni[i]+1] = norms[idx+1];
                    model.meshes[i].normals[mni[i]+2] = norms[idx+2];
                    mni[i]+=3;
                }
            }
            fi = nextNode;
        }
    }
    
    // upload all the meshes to the GPU
    model.transform = MatrixIdentity();
    for (int i = 0; i < materialCount; i++) {
        //rlLoadMesh(&model.meshes[i], false);
        UploadMesh(&model.meshes[i], false);
    }

    // restore current directory (changed for mtl and texture loading)
    CHDIR(currentDir);
    
    // get rid of the linked list of face indexes
    releaseFaces();
    
    RL_FREE(objdata);
    RL_FREE(mtldata);

    return model;
}
