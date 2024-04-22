#~ # >>> CHANGE <<< per project
PROJ_NAME = HW2
# Typing 'make' or 'make $(PROJ_NAME)' will create the executable file.

# makefile tutorial: https://www.cs.swarthmore.edu/~newhall/unixhelp/howto_makefiles.html

# ~~ Compiler Specification ~~
CC = gcc

# ~~ Options and Flags ~~
#  Win
ifeq "$(OS)" "Windows_NT"
CFLAGS=-O3 -Wall 
LIBS=-lglut32cu -lglu32 -lopengl32
CLEAN=del *.exe *.o *.a *~
else
#  OSX
ifeq "$(shell uname)" "Darwin"
CFLAGS=-O3 -Wall -Wno-deprecated-declarations 
LIBS=-framework GLUT -framework OpenGL
#  Linux/Unix/Solaris
else
CFLAGS=-O3 -Wall 
LIBS=-lglut -lGLU -lGL -lm
endif
#  OSX/Linux/Unix/Solaris
CLEAN=rm -f $(EXE) *.o *.a *~
endif

# ~~ Default Recipe Name ~~
# typing 'make' will invoke the first target entry in the file  (in this case the default target entry)
# you can name this target entry anything, but "default" or "all" are the most commonly used names by convention
default: $(PROJ_NAME)

# ~~ Top Recipe ~~
# To create the executable file $(PROJ_NAME) we need the object files
# NOTE: The executable recipe is defined before all of the sub-recipes
$(PROJ_NAME): OGL_utils.o $(PROJ_NAME).o
	$(CC) $(CFLAGS) -o $(PROJ_NAME) OGL_utils.o $(PROJ_NAME).o $(LIBS)

# ~~ Dependency Recipes ~~
# To create the object file for the dependency, we need the source
OGL_utils.o: OGL_utils.c
	$(CC) $(CFLAGS) -c OGL_utils.c $(LIBS)

# ~~ clean Recipe ~~
# To start over from scratch, type 'make clean'.  This removes the executable file, as well as old .o objects files and *~ backup files:
clean: 
	$(CLEAN)
