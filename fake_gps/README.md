# FAKE GPS

process current pose and the publish to gnss pose  
  
only publish to gnss pose when the ndt status is ok  
  
convert pose to tf and publish to /tf for visualization  

## Require: 
1. ndt_matching
2. ndt_matching_monitor  
3. vel_pose_connect
  
## Attention!
The node must be off before the robot is initialized at the start  
The sequence is like the follow:  
1. Turn on TF, map, and voxel_grid_filter  
2. Turn on ndt_matching, ndt_matching_monitor, and vel_pose_connect  
3. Wait for the robot to be localized properly at the initial position  
4. Run ```rosrun fake_gps fake_gps_node``` after the robot is localized properly  
  

If the robot is not localized or localized in wrong position, the fake_gps output will be wrong  
This make the ndt_matching keep rematching in the wrong position  
To fix this, turn off fake_gps node first, then restart ndt_matching and ndt_matching_monitor  
  
![init points](/img/maps-init.png)
  
Point A: 0, 0, 0, 0, 0, 0  
Point B: 7.5, 13.5, -0.6, 0, 0, -2.732
  

![structure](/img/structure.png)
