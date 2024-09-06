########## COMPILATION SETTINGS ####################################################################

CFLG=-O3 -Wall -std=gnu17
LIBS=-lglut -lGLU -lGL -lm
CLEAN=rm -f *.out *.o *.a


########## PROGRAMS ################################################################################

00_character.o: 00_character.c geometry.h toolbox.h # Rolling geometric tank, Mouse Control
# 01_Planet-Geom.o: 01_Planet-Geom.c geometry.h toolbox.h # --- Swirling atmosphere, Compute shader
# 02_VBO_VNT_f.o: 02_VBO_VNT_f.c toolbox.h # ------------------ Vertex Buffer Objects, Nested
# 03_Tetra-Tank_js.o: 03_Tetra-Tank_js.c geometry.h toolbox.h # Rolling geometric tank, Joystick control
# 04_Sparkle.o: 04_Sparkle.c geometry.h toolbox.h # ----------- Shader dev for 03 tank
# 05_Chopperhedron.o: 05_Chopperhedron.c geometry.h toolbox.h # Transforming tank: Land + Air
# 06_Behavior-Tree.o: 06_Behavior-Tree.c geometry.h toolbox.h # Behavior Tree (BT) dev + testing
# 07_BT-Animation.o: 07_BT-Animation.c geometry.h toolbox.h # - BT keyframe animation test



########## DEPENDENCIES ############################################################################

### OpenGL Toolbox ###
OGL_utils.o: OGL_utils.c toolbox.h # ----- Window management and rendering
load_assets.o: load_assets.c toolbox.h # - Loading textures, etc.
vector-f_ops.o: vector-f_ops.c toolbox.h # Vector Algebra
shaders.o: shaders.c toolbox.h # --------- Loading and compiling shader programs
behavior.o: behavior.c toolbox.h # ------- Behavior Trees

### Geometry Construction ###
matrix4x4f.o: matrix4x4f.c matrix4x4f.h # ------------------ Linear Algebra + Projections
transform.o: transform.c geometry.h toolbox.h matrix4x4f.h # Homogeneous Coordinates
TriNet.o: TriNet.c geometry.h toolbox.h matrix4x4f.h # ----- Simple Meshes
VNCT_f.o: VNCT_f.c geometry.h toolbox.h # ------------------ Vertex Buffer Objects, based on meshes
draw_geo.o: draw_geo.c geometry.h toolbox.h # -------------- Render a triangle at a time, inefficiently



########## ARCHIVES & GENERAL RULES ################################################################

toolbox.a: OGL_utils.o load_assets.o vector-f_ops.o shaders.o behavior.o # OpenGL Toolbox
	ar -rcs $@ $^

geometry.a: transform.o TriNet.o VNCT_f.o draw_geo.o matrix4x4f.o # Geometry Construction
	ar -rcs $@ $^

# Compile rules
.c.o:
	gcc -c $(CFLG)  $<



########## PROGRAM & CLEAN RULES ###################################################################


##### Program Rules #######################################################

00_character: 00_character.o geometry.a toolbox.a 
	gcc $(CFLG) -o $@.out $^ $(LIBS)

# 01_Planet-Geom: 01_Planet-Geom.o geometry.a toolbox.a 
# 	gcc $(CFLG) -o $@.out $^ $(LIBS)

# 02_VBO_VNT_f: 02_VBO_VNT_f.o matrix4x4f.o toolbox.a 
# 	gcc $(CFLG) -o $@.out $^ $(LIBS)

# 03_Tetra-Tank_js: 03_Tetra-Tank_js.o geometry.a toolbox.a 
# 	gcc $(CFLG) -o $@.out $^ $(LIBS)

# 04_Sparkle: 04_Sparkle.o geometry.a toolbox.a 
# 	gcc $(CFLG) -o $@.out $^ $(LIBS)

# 05_Chopperhedron: 05_Chopperhedron.o geometry.a toolbox.a 
# 	gcc $(CFLG) -o $@.out $^ $(LIBS)

# 06_Behavior-Tree: 06_Behavior-Tree.o geometry.a toolbox.a 
# 	gcc $(CFLG) -o $@.out $^ $(LIBS)

# 07_BT-Animation: 07_BT-Animation.o geometry.a toolbox.a 
# 	gcc $(CFLG) -o $@.out $^ $(LIBS)

##### Clean Rule ##########################################################

#  Clean
clean:
	$(CLEAN)
