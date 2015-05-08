Steps for running the GPS_common node

1.In order to run the GPS_Common UTM_odometery node it needs to subscribe to a topic 'fix' which can  be published by GPSD_client package. 

2. GPSD_Client doesn't allow the use of other USB device. Hence use the nmea_navsat_driver (ROS package) to get the 'fix' topic.