shopt -s  extglob
rm -rf !(make.sh)
cmake ..
make
rm -rf CMakeFiles CMakeCache.txt cmake_install.cmake Makefile