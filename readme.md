This node converts point cloud data to a drivable area.
This method considers objects detected by CNN, so the drivable area covers areas occupied by objects detected to prevent inefficient planning.

Input msg:
  <arg name="input_pointcloud2_topic" default="/middle/rslidar_points" />
  <arg name="input_obs_topic" default="/zzz/perception/objects_tracked" />
  <arg name="input_egopose_topic" default="/zzz/navigation/ego_pose" />
  <arg name="input_tf_topic" default="/tf" />

output msg:
  <arg name="output_gridmap_topic" default="/zzz/perception/grid_map" />
  <arg name="output_gridmap_obs_topic" default="/zzz/perception/grid_map_obs" />

there are two outputs. 
output_gridmap_topic is the raw drivable area without considering objects, which means the area occupied by these objects is considered undrivable. This msg mainly for comparation. 
output_gridmap_obs_topic is the drivable area considering objected detected as aforementioned.
both are GridMapStamped.msg

related codes:
	visualization.
	collision_check with drivable area.
	message files.

need optimizing:
1. expensions used in object projection. 
  fixed expension: (related to class) in gridmapconstructor.cpp
  scaling expesion: (a fixed expension time) in rsgridmap.cc
2. slope and curb detection.
  This method usually fails in curb detection for its low height. Some curb may be mistakely regarded drivable. Mostly it may not affect the safety, for this method is often used as a support not a main basis.
3. an intuitive method for collision checking
4. No header in the output message

bugs:

1. This node accepts four messages without checking their time label. It may cause error.
In ros, when the vehicle changes the direction of its velocity quickly (usually with a low velocity), the system may give a wrong drivable area. I dont know if it results from mismatching of the time labels.

2. in rsgridmap.h
  TODO: the preprocess function need optimizing
  1 point just in front of the car (seems the point reflected by the top of the car) is filtered but sometimes it failed, which causes the ego vehicle brake.
  2 to decrease the number of points, points with a height(z) too high or too low will be filtered but it may fail when slope
