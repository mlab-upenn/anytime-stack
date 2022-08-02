
echo "Compiling Code...";

g++ -O3 -march=native -mtune=intel -std=c++17 -mavx2 -mfma -flto -fopenmp -fPIC -fno-math-errno -I/usr/local/include/eigen3/ $1.cpp -o $1 -lstdc++;

echo "Compile Done, running code...";

./$1


