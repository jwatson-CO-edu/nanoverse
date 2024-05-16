########## COMPILATION SETTINGS ####################################################################

CFLG=-O3 -Wall -std=gnu17
LIBS=-lglut -lGLU -lGL -lm
CLEAN=rm -f *.out *.o *.a


########## PROGRAMS ################################################################################

00_Dyn-6D.o: 00_Dyn-6D.c toolbox.h # ------------------------ 6-Dimensional Lorenz Attractors
01_Planet-Geom.o: 01_Planet-Geom.c geometry.h toolbox.h # --- Swirling atmosphere, Compute shader
02_VAO_VNC_f.o: 02_VAO_VNC_f.c geometry.h toolbox.h # ------- Vertex Array Objects, Nested
03_Tetra-Tank_js.o: 03_Tetra-Tank_js.c geometry.h toolbox.h # Rolling geometric tank, Joystick control



########## DEPENDENCIES ############################################################################

### Float Matrix Math ###
matrix4x4f.o: matrix4x4f.c matrix4x4f.h

### OpenGL Toolbox ###
OGL_utils.o: OGL_utils.c toolbox.h
load_assets.o: load_assets.c toolbox.h
vector-f_ops.o: vector-f_ops.c toolbox.h
shaders.o: shaders.c toolbox.h

### Geometry Construction ###
transform.o: transform.c geometry.h toolbox.h matrix4x4f.h
TriNet.o: TriNet.c geometry.h toolbox.h 
VAO_VNC_f.o: VAO_VNC_f.c geometry.h toolbox.h 
draw_geo.o: draw_geo.c geometry.h toolbox.h 



########## ARCHIVES & GENERAL RULES ################################################################

toolbox.a: OGL_utils.o load_assets.o vector-f_ops.o shaders.o
	ar -rcs $@ $^

geometry.a: transform.o TriNet.o VAO_VNC_f.o draw_geo.o
	ar -rcs $@ $^

# Compile rules
.c.o:
	gcc -c $(CFLG)  $<



########## PROGRAM & CLEAN RULES ###################################################################


##### Program Rules #######################################################

00_Dyn-6D: 00_Dyn-6D.o toolbox.a
	gcc $(CFLG) -o $@.out $^ $(LIBS)

01_Planet-Geom: 01_Planet-Geom.o geometry.a toolbox.a 
	gcc $(CFLG) -o $@.out $^ $(LIBS)

02_VAO_VNC_f: 02_VAO_VNC_f.o geometry.a toolbox.a matrix4x4f.o
	gcc $(CFLG) -o $@.out $^ $(LIBS)

03_Tetra-Tank_js: 03_Tetra-Tank_js.o geometry.a toolbox.a matrix4x4f.o
	gcc $(CFLG) -o $@.out $^ $(LIBS)


##### Clean Rule ##########################################################

#  Clean
clean:
	$(CLEAN)
