/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <gps_common/position.h>



using namespace gps_common;
Position::Position3D odom_3d;
static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;
double rot_cov;

bool GlobalToLocal(Position::Pose3D *current)
{
    static Position::Pose3D map_origin;
    static bool first_pose_received = false;

    ROS_INFO("Global data (%.3f, %.3f, %.3f) (%.3f, %.3f, %.3f)",
    current->x, current->y, current->z,
    current->roll, current->pitch, current->yaw);

    bool initial_pose = !first_pose_received;

    if(initial_pose)
    {
        map_origin   = *current;
        map_origin.x = map_origin.x;              //rint(map_origin.x/100000.0) * 100000.0;
        map_origin.y = map_origin.y;             //rint(map_origin.y/100000.0) * 100000.0;
        map_origin.z = map_origin.z;

        first_pose_received = true;

        ROS_INFO("INITIAL data (%.3f, %.3f, %.3f), map origin (%.3f, %.3f, %.3f)",
        current->x, current->y, current->z,
        map_origin.x, map_origin.y, map_origin.z);

    }

    current->x -= map_origin.x;
    current->y -= map_origin.y;
    current->z -= map_origin.z;

    ROS_INFO("Local data  (%.3f, %.3f, %.3f) (%.3f, %.3f, %.3f)",
    current->x, current->y, current->z,
    current->roll, current->pitch, current->yaw);

    return initial_pose;
}

bool ConvertToLocal(Position::Pose3D *current)
{

    if(GlobalToLocal(current))
            return false;
}

void callback(const sensor_msgs::NavSatFixConstPtr& fix) {
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_INFO("No fix.");
    return;
  }

  if (fix->header.stamp == ros::Time(0)) {
    return;
  }

  double  northing_double , easting_double;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing_double, easting_double, zone);
  int easting = easting_double;
  int northing = northing_double;
  int altitude = fix->altitude;

  if (odom_pub) {

    odom_3d.pos.x = easting;
    odom_3d.pos.y = northing;
    odom_3d.pos.z = altitude;

    odom_3d.pos.roll = 0;
    odom_3d.pos.pitch = 0;
    odom_3d.pos.yaw = 0;

    ConvertToLocal(&odom_3d.pos);

    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    if (frame_id.empty())
      odom.header.frame_id = fix->header.frame_id;
    else
      odom.header.frame_id = frame_id;

    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = odom_3d.pos.x;
    odom.pose.pose.position.y = odom_3d.pos.y;
    odom.pose.pose.position.z = odom_3d.pos.z;


    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 1;
    
    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{
      fix->position_covariance[0],
      fix->position_covariance[1],
      fix->position_covariance[2],
      0, 0, 0,
      fix->position_covariance[3],
      fix->position_covariance[4],
      fix->position_covariance[5],
      0, 0, 0,
      fix->position_covariance[6],
      fix->position_covariance[7],
      fix->position_covariance[8],
      0, 0, 0,
      0, 0, 0, rot_cov, 0, 0,
      0, 0, 0, 0, rot_cov, 0,
      0, 0, 0, 0, 0, rot_cov
    }};

    odom.pose.covariance = covariance;

    odom_pub.publish(odom);
    
  }
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");
  ros::Rate loop_rate(1);   


  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);

  odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);

  ros::Subscriber fix_sub = node.subscribe("fix", 10, callback);
  

  ros::spin();
  loop_rate.sleep();
 


}

