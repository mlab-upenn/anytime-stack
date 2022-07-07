REM g++ -O3 -march=native -mtune=intel -mavx2 -mfma -flto -fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++17 -ID:/Downloads/Academic_Tools/eigen-3.4.0/Eigen3 %1.cpp -o %1

REM g++ -O3 -march=native -mtune=intel -DNDEBUG -std=c++17 -mavx2 -mfma -flto -fopenmp -fPIC -ID:/Downloads/Academic_Tools/eigen-3.4.0/ %1.cpp -o %1

g++ -O3 -march=native -mtune=intel -std=c++17 -mavx2 -mfma -flto -fopenmp -fPIC -fno-math-errno -ID:/Downloads/Academic_Tools/eigen-3.4.0/ -ID:/Documents/Summer_22/UPenn_Research/LBFGSpp/include/ %1.cpp -o %1

REM g++ -O3 -march=native -mtune=intel -std=c++17 -mavx2 -mfma -flto -fopenmp -fPIC -ID:/Documents/Local_Repositories/eigen %1.cpp -o %1

REM g++ -ID:/Downloads/Academic_Tools/eigen-3.4.0/ %1.cpp -o %1

.\%1.exe