CFLG=-O3 -Wall -std=gnu17
LIBS=-lglut -lGLU -lGL -lm
CLEAN=rm -f *.out *.o *.a


# Prog Dependencies
00_Dyn-6D.o: 00_Dyn-6D.c toolbox.h

# Lib Dependencies
toolbox.h: matrix4x4f.h
vector-f_ops.o: vector-f_ops.c toolbox.h
OGL_utils.o: OGL_utils.c toolbox.h
matrix4x4f.o: matrix4x4f.c toolbox.h



#  Create archive
toolbox.a:vector-f_ops.o OGL_utils.o vector-f_ops.o matrix4x4f.o
	ar -rcs $@ $^

# Compile rules
.c.o:
	gcc -c $(CFLG)  $<

#  Compile shaders
#  Link
00_Dyn-6D:00_Dyn-6D.o toolbox.a
	gcc $(CFLG) -o $@.out $^  $(LIBS)

#  Clean
clean:
	$(CLEAN)