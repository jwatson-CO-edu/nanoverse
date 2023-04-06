
APPNAME:=$(shell basename `pwd`)

#INSTR:= -fsanitize=leak -fsanitize=address -fno-omit-frame-pointer -fsanitize=undefined
INSTR:= -fsanitize=address,leak,undefined,pointer-compare,pointer-subtract
INSTR+= -fno-omit-frame-pointer

LDFLAGS:=-L ../raylib -lm -lraylib -lX11 -ldl -pthread

CFLAGS:= -Wfatal-errors -pedantic -Wall -Wextra -Werror
CFLAGS+= -std=c99 -I ./include -I ../raylib/src

SRC:=$(wildcard src/*.c)
OBJ:=$(SRC:src/%.c=obj/%.o)
INC:=$(wildcard include/*.h)

CC=gcc

$(APPNAME): $(OBJ)
	$(CC) $(OBJ) -o $(APPNAME) $(LDFLAGS)

$(OBJ): obj/%.o : src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: debug release
debug: CFLAGS+= -g $(INSTR)
debug: LDFLAGS+= $(INSTR)

release: CFLAGS+= -O3

debug release: clean $(APPNAME)

.PHONY:	clean
clean:
	rm obj/* -f
	rm $(APPNAME) -f

style: $(SRC) $(INC)
	astyle -A10 -s4 -S -p -xg -j -z2 -n src/* include/*
