########## COMPILATION SETTINGS ####################################################################

CFLG=-O3 -Wall -std=gnu17
LIBS=-lglut -lGLU -lGL -lm
CLEAN=rm -f *.out *.o *.a



########## DEPENDENCIES ############################################################################

### Prog Dependencies ###
00_Dyn-6D.o: 00_Dyn-6D.c toolbox.h
01_Planet-Geom.o: 01_Planet-Geom.c toolbox.h geometry.h

### Float Matrix Math ###
matrix4x4f.o: matrix4x4f.c matrix4x4f.h

### OpenGL Toolbox ###
OGL_utils.o: OGL_utils.c toolbox.h
load_assets.o: load_assets.c toolbox.h
vector-f_ops.o: vector-f_ops.c toolbox.h

### Geometry Construction ###
TriNet.o: TriNet.c toolbox.h geometry.h



########## ARCHIVES & GENERAL RULES ################################################################

toolbox.a: OGL_utils.o load_assets.o vector-f_ops.o
	ar -rcs $@ $^

geometry.a: toolbox.a TriNet.o
	ar -rcs $@ $^

# Compile rules
.c.o:
	gcc -c $(CFLG)  $<



########## PROGRAM & CLEAN RULES ###################################################################

##### Program Rules #######################################################

00_Dyn-6D: 00_Dyn-6D.o toolbox.a
	gcc $(CFLG) -o $@.out $^ $(LIBS)

01_Planet-Geom: 01_Planet-Geom.o toolbox.a geometry.a
	gcc $(CFLG) -o $@.out $^ $(LIBS)


##### Clean Rule ##########################################################

#  Clean
clean:
	$(CLEAN)
