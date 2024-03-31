shopt -s  extglob
# rm -rf !(make.sh)
cmake -DCMAKE_BUILD_TYPE=Release ..
make
# rm -rf CMakeFiles CMakeCache.txt cmake_install.cmake Makefile