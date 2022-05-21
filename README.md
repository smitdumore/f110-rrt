# f110-rrt
rrt path planning in f1tenth simulator
![Screenshot from 2022-05-21 18-53-07](https://user-images.githubusercontent.com/75038294/169653809-25344bc8-3f2a-4501-aac1-3b35c961e766.png)

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
