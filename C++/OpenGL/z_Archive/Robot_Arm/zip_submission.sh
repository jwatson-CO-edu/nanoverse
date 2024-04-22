#!/bin/bash

# zip -r /home/admin/download.zip ./*
# https://superuser.com/a/354657
#find some/dir -type f -name '*.txt' -print | zip ... -@ ...

#~ echo ( find -type f | grep -P '\.c|\.h|\.cpp' )

NAME=HW3

# Zip only matching files
# find -type f | grep -P '\.c|\.h|\.cpp|\.txt|\.pdf|makefile' | zip $NAME.zip -@

# Recursive zip only matching files , https://stackoverflow.com/a/1112537
zip -R $NAME.zip '*.c' '*.h' '*.cpp' '*.txt' '*.pdf' 'makefile' '*.sh'
