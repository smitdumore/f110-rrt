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

how to run:


description:
This project is a racing strategy for autonomous racing for the f1tenth platform. This approach uses a offline pre-computed trajectory around the racetrack for Global Planning. The goal of this project is to develop a RRT local planner to compete head to head with opponents on the race track.

RRT- This is a classic sampling based motion planning method .The RRT algorithm though faster than grid based approached , provided non-smooth trajectories which were difficult to track using a pure pursuit controller.

Thus RRT star was used which is proven to provide completeness and optimality guarantees. But still the trajectories were made up of straight lines that did not take into account the dynamics of the car. Thus with a pure pursuit controller , tracking the RRT star trajectory  became difficult and would result in an occasional collision.  Also for this algorithm to run real time the sampling iterations have to be reduced , which means it will not create optimal trajectories. 

Also due to the random nature of the algorithm , the trajectory can rapidly switch between lidar scan updates , resulting in unwanted behaviours. Thus once a path is found it was freezed for a certain distance. Also to smooth out the path I used c++ interpolate to fit a regularised cubic spline  spline between the final path samples

