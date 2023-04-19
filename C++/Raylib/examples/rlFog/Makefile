
APPNAME = simple_fog

LDFLAGS:=-L ../raylib/src -lm -lraylib -lX11 -ldl -pthread

CFLAGS:= -Wfatal-errors -pedantic -Wall -Wextra -Werror
CFLAGS+= -std=c99 -I ./include -I ../raylib/src -DPLATFORM_DESKTOP

SRC:=$(wildcard src/*.c)
OBJ:=$(SRC:src/%.c=obj/%.o)
INC:=$(wildcard include/*.h)

CC=gcc

all: $(APPNAME)

$(APPNAME): $(OBJ)
	$(CC) $(OBJ) -o $(APPNAME) $(LDFLAGS)

$(OBJ): obj/%.o : src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: debug release
debug: CFLAGS+= -g
release: CFLAGS+= -O3

debug release: clean $(APPNAME)

.PHONY:	clean
clean:
	rm obj/* -f
	rm $(APPNAME) -f


