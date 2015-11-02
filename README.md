# gps_umd
The difference between this package and the original one is that is start point co-ordinates are zero. 
i.e when you launch the node x =0, y= 0 and Z = 0

Steps for running the GPS_common node

1. The node can be launched by:  
rosrun gps_common utm_odometry_node

2.In order to run the GPS_Common UTM_odometery node, it needs to subscribe to a topic 'fix' which can be published by GPSD_client package.
  GPSD_Client doesn't allow the use of other USB device. Hence use the nmea_navsat_driver (ROS package) to get the 'fix' topic.

