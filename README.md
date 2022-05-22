# f110-rrt
rrt path planning in f1tenth simulator
![Screenshot from 2022-05-22 10-05-36](https://user-images.githubusercontent.com/75038294/169678895-97a2faa0-5dca-4f59-ad21-c449f73d93bc.png)

#installing xtensor
1. conda install -c conda-forge xtensor

#installing xtensor interpolate
1. git clone git@github.com:rjsberry/xtensor-interpolate.git
2. cd xtensor-interpolate
3. mkdir build && cd build
4. sudo apt install gfortran
5. cmake -G "Unix Makefiles" ..
6. cmake --build .
7. sudo make install
