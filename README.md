# Fun with Ceres and SLAM
I created this repo to try out the [Ceres](http://ceres-solver.org/) optimisation library.

The data is being optimised is a (very simply) simulated trajectory of some vehicle with odometry and perhaps other sensor measurements.

Noise, drift and various loop closure options are available.

## Requirements
[Ceres](http://ceres-solver.org/) and [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html)
```shell script
sudo apt-get install libceres-dev libeigen3-dev
```
(alternatively you can use/install from source)

## Build
```shell script
mkdir build
cd build
cmake ..
make
```


## Run
```shell script
./build/simslam
```
Visualise results
```shell script
python plot_results.py ./build
```

## Notes
A. With no noise, drift or loop closures.  
The initial state of the trjaectory is exactly the true trajectory, and there is noting for the optimiser to do.
```shell script
./build/simslam 0 0 0 0 0
```
![](images/plot0000.jpg)
___
B. With a little drift.  
Now the initial values of the nodes in the trajectory are not correct (except the first), and the end position is clearly well off where it should be. 
In this case the only information available in the optimisation is a linear sequence of odometry measurements which is all self-consistent i.e. there's no other information to contradict the odometry, which is why the optimisation cannot fix this.
```shell script
./build/simslam 0 1 0 0 0
```
![](images/plot0100.jpg)
___
C. With drift and a loop closure observation (yellow line) between the start and end of the trajectory.  
Now we have an additional measurement between the start and end of the trajectory that is inconsistent with the odometry information. The optimisation is now able to 'correct' the errors in odometry. However, some 'warping' of the true trajectory remains (due to drift error in the translation element of the odometry measurement) which the single loop closure cannot fix, because it provides no additional information about the rest of the trajectory.   
```shell script
./build/simslam 0 1 1 0 0
```
![](images/plot0110.jpg)
___
D. With a few more loop closures.  
Now the warping is fixed - of course in practice it may not always be possible to generate such constraints exactly where we want.
```shell script
./build/simslam 0 1 2 0 0
```
![](images/plot0120.jpg)
___
E. Add some noise as well for a marginal increase in realism. This only adds noise to the x and y components of the relative motion.
```shell script
./build/simslam 1 1 2 0 0
```
![](images/plot1120.jpg)
___
F. Add noise to all components of the relative motions (translational and rotational).
```shell script
./build/simslam 2 1 2 0 0
```
![](images/plot2120.jpg)
___
G. Increase the noise! These (deliberately convenient) loop closures are quite effective.
```shell script
./build/simslam 3 1 2 0 0
```
![](images/plot3120.jpg)
___
H. Adding orientation edges to each node can also improve the optimised trajectory. 
This is some measurement of absolute orientation e.g. as we might get from an IMU (including magnetometer for heading).
In this example loop closure is removed completely.
```shell script
./build/simslam 0 1 0 1 0
```
![](images/plot0101.jpg)
___
I. It's quite effective even when there is a lot of noise on the relative motion and orientation measurements.
```shell script
./build/simslam 3 1 0 1 0
```
![](images/plot3101.jpg)
___
J. If the absolute heading of an orientation reading can't be used e.g. 
IMU does not contain a magnetometer or another way to maintain an absolute heading,
then we can just use the pitch and roll elements. Here this is achieved by constructing a 'gravity vector' which defines a horizontal plane that the nodes should be aligned to.
This is probably easiest to see working with just some drift. The optimised trajectory still contains drift due to xy movement and yaw, but is (correctly) flattened down to the horizontal plane (although that's not completely clear from the 3d plot).  
```shell script
./build/simslam 2 0 0 2 0
```
![](images/plot0202.jpg)
___
K. Add an absolute position estimate (e.g. like GNSS) at the start only  
In this example, the frame in which the absolute position estimated is specified to be a bit different the default (identity) so that the effects of using these measurements is more obvious.  
Because we only added have a single position measurement at the starting point, so although the start is correctly shifted, no information is available about the orientation of the trajectory within this absolute frame.     
```shell script
./build/simslam 0 0 0 0 1
```
![](images/plot00001.jpg)
___
L. Add an additional absolute position estimate near the middle    
Now that there are two absolute position measurements, we can infer the orientation and the corrected trajectory is both translated and orientated correctly.     
```shell script
./build/simslam 0 0 0 0 2
```
![](images/plot00002.jpg)


## TODO
Add more (simulated) sensor information...
 - Add specific information matrix for absolute position estimates
 - Offset IMU and GNSS measurement frames, and then estimate (refine as part of optimisation)
 - Nicer input arguments!
 - Tidy noise generation parts
 - More sensible covariance/information matrix values
 - Allow visualisation of other edges (may want to change visualisation approach entirely)
 - Qualitative evaluation of optimised results vs ground truth
 