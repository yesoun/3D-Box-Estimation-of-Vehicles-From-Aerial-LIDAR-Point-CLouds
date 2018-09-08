# 3D-Box-Estimation-of-Vehicles-From-Aerial-LIDAR-Point-Clouds
Using LIBLAS we find the estimate positions of vehicle from Aerial LIDAR scans mounted on a Drone flying over 
the top of the road. The idea is not based on 3D CNNs, but it is rather based on trimming the 3D search space 
by excluding any set of point cloud that does not belong to the road, then downsampling of PC using 3D Voxelization
and move a 3D fix-sized box (at the beginning) and search the ones that have the highest occupancy.   
