#!/bin/bash

# zip -r /home/admin/download.zip ./*
# https://superuser.com/a/354657
#find some/dir -type f -name '*.txt' -print | zip ... -@ ...

#~ echo ( find -type f | grep -P '\.c|\.h|\.cpp' )

NAME=HW6

# Zip only matching files
# find -type f | grep -P '\.c|\.h|\.cpp|\.txt|\.pdf|makefile' | zip $NAME.zip -@

# Recursive zip only matching files , https://stackoverflow.com/a/1112537
# zip -R $NAME.zip '*.c' '*.h' '*.cpp' '*.txt' '*.pdf' 'makefile' '*.sh' '*.MINPACK' '*.LGPL' '*.MPL2' '*.README' 'INSTALL' '*.md' '*.in' '*.GPL' '*.BSD' 'signature_of_eigen3_matrix_library' 'README' 'COPYING' '*.hh' '*.cmake' 'go_mean' '*.cxx' '*.html' '*.js' '*.cc' '*.cu' '*.f' '*.py' '*.dat' '*.natvis' '*.png' '*.dox' '*.css'

# Recursive zip , excluding slected directories
zip -r $NAME.zip * -x test/\* build/\* eigen/.hg/\*
# There are too many types and names in the Eigen lib to match them all.  Many files do not have extensions
