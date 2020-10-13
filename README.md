What do I really want to do with this repo?
Maybe it would be nice to have a little drivable vehicle? But then it's getting more complicated and away from the initial idea of investigating Ceres.
I think I should stick to that for the moment.

Steps:
1. Basic loop, no noise, optimise, output and visualise (python script) - basically check everything is roughly working
2. Add noise
3. Add scaling of residuals by covariance
4. Fix first node
5. Add absolute orientation constraints
6. Add gravity vector instead (no yaw information)



Maybe it can start very simple but then I can start adding things like gravity vector or absolute orientation etc.


I'm unclear why the xyz components of the quaternion are a good choice for the residuals - I mean, it seems 'seem' suprising, but I'm just not clear exactly why this works.

I'm also unsure why only the covariance matrix for EDGE_SE3:QUAT only has components relating to xyz, not w. I guess it doesn't need all 4, but then I'm still unsure exactly why.

WORKING NOTES:
