CFLG=-O3 -Wall
LIBS=-lglut -lGLU -lGL -lm
CLEAN=rm -f *.out *.o *.a

# Dependencies
OGL_base.o: OGL_base.c OGL_base.h
matrix4x4.h: OGL_base.h
toolbox.h: OGL_utils.h

OGL_utils.o: OGL_utils.c OGL_utils.h