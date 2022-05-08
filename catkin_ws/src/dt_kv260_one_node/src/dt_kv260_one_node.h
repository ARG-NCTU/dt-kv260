#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <vector>
#include <string>
#include <math.h>
#include <signal.h>
#include <Eigen/Geometry>
#include <iterator>
#include <list>

// ZED
#include "videocapture.hpp"
#include "sensorcapture.hpp"
#include "ocv_display.hpp"
#include "calibration.hpp"
#include "stopwatch.hpp"
#include "stereo.hpp"
#include <iomanip>
#include <opencv2/opencv.hpp>

// Ros
#include <ros/ros.h>
#include <ros/timer.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

// cv_bridge
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Pcl load and ros
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

using namespace ros;
using namespace pcl;
using namespace std;

struct point{
  float x;
  float y;
  float z;
};

struct laser{
  float ranges[241];
};

struct cmd_vel{
  float linear_x;
  float angular_z;
};



class DT_kv260_Node{
public:
  DT_kv260_Node();
  cv::Mat stereo_matching(const sl_oc::video::Frame frame,
                          sl_oc::tools::StereoSgbmPar stereoPar,
                          cv::Ptr<cv::StereoSGBM> left_matcher,
                          cv::Mat map_left_x,
                          cv::Mat map_left_y,
                          cv::Mat map_right_x,
                          cv::Mat map_right_y);

  laser create_depth_and_points_and_laser(cv::Mat left_disp_float, double baseline, double fx, double fy, double cx, double cy);
  cmd_vel obstacle_avoidance(laser);
  void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy);

private:
  // All for ROS
  NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher pub_left;
  ros::Publisher pub_pc, pub_laser, pubSpeed;
  ros::Subscriber subJoystick;
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg; // >> message to be sent
  std_msgs::Header header_camera; // empty header
  bool flag = 0;


};
