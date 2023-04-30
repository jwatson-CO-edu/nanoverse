#!/bin/bash

NAME=HW2

find -type f | grep -P '\.c|\.h|\.cpp|\.txt|\.pdf|makefile' | zip $NAME.zip -@
