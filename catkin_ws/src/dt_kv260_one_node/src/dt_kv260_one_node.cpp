#include "dt_kv260_one_node.h"

cv::Mat DT_kv260_Node::stereo_matching(const sl_oc::video::Frame frame,
                                        sl_oc::tools::StereoSgbmPar stereoPar,
                                        cv::Ptr<cv::StereoSGBM> left_matcher,
                                        cv::Mat map_left_x,
                                        cv::Mat map_left_y,
                                        cv::Mat map_right_x,
                                        cv::Mat map_right_y){
  cv::Mat frameYUV, frameBGR, left_raw, left_rect, right_raw, right_rect, left_for_matcher, right_for_matcher, left_disp_half,left_disp,left_disp_float, left_disp_vis;
  // ----> Conversion from YUV 4:2:2 to BGR for visualization
  frameYUV = cv::Mat( frame.height, frame.width, CV_8UC2, frame.data );
  cv::cvtColor(frameYUV,frameBGR,cv::COLOR_YUV2BGR_YUYV);
  // <---- Conversion from YUV 4:2:2 to BGR for visualization
  // ----> Extract left and right images from side-by-side
  left_raw = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
  right_raw = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));
  // <---- Extract left and right images from side-by-side


  // ----> Apply rectification
  cv::remap(left_raw, left_rect, map_left_x, map_left_y, cv::INTER_AREA );
  cv::remap(right_raw, right_rect, map_right_x, map_right_y, cv::INTER_AREA );
  // <---- Apply rectification
  //
  // // ----> Stereo matching
  left_for_matcher = left_rect; // No data copy
  right_for_matcher = right_rect; // No data copy
  // Apply stereo matching
  left_matcher->compute(left_for_matcher, right_for_matcher,left_disp_half); //left_disp_half->CV_16UC1
  left_disp_half.convertTo(left_disp_float,CV_16UC1);
  cv::multiply(left_disp_float,1./16.,left_disp_float); // Last 4 bits of SGBM disparity are decimal
  // <---- Stereo matching



  //ROS
  img_bridge = cv_bridge::CvImage(header_camera, sensor_msgs::image_encodings::BGR8 , left_raw);
  img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
  pub_left.publish(img_msg);
  return left_disp_float;
}

laser DT_kv260_Node::create_depth_and_points_and_laser(cv::Mat left_disp_float,
                                                    double baseline, double fx, double fy, double cx, double cy){

  // ----> Extract Depth map
  // The DISPARITY MAP can be now transformed in DEPTH MAP using the formula
  // depth = (f * B) / disparity
  // where 'f' is the camera focal, 'B' is the camera baseline, 'disparity' is the pixel disparity
  cv::Mat left_depth_map;
  double num = static_cast<double>(fx*baseline);
  cv::divide(num,left_disp_float,left_depth_map);
  // <---- Extract Depth map

  // ----> Create Point Cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; //ROS
  cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>); //ROS

  float tf_matrix[4][4] ={{ 0.939753,         0,  0.341854,         0},
                          {        0,         1,         0,         0},
                          {-0.341854,         0,  0.939753,         0},
                          {        0,         0,         0,         1}};

  list<point> pc_create;
  size_t buf_size = static_cast<size_t>(left_depth_map.cols * left_depth_map.rows);

  // pointcloud_to_laserscan
  double tolerance_=0.01;
  double min_height_ = -0.1;
  double max_height_ = 10;
  double angle_min_ = -2.094395;
  double angle_max_ = 2.094395;
  double angle_increment_ = 0.017453;
  double scan_time_ = 0.1;
  double range_min_ = 0;
  double range_max_ = 100;

  sensor_msgs::LaserScan laser_output; //ROS
  laser_output.ranges.assign(241, range_max_); //ROS

  laser ls;
  for(int i=0;i<sizeof(ls.ranges)/sizeof(ls.ranges[0]);i++) ls.ranges[i]=range_max_;

  for(size_t idx=0; idx<buf_size;idx++ ){
    size_t r = idx/left_depth_map.cols;
    size_t c = idx%left_depth_map.cols;
    ushort depth = left_depth_map.at<ushort>(r, c); //left_depth_map is CV_16U
    if(!isinf(depth) && depth >0 && depth > 300 && depth < 10000) //stereoPar.minDepth_mm=300, stereoPar.maxDepth_mm=10000
    {
        ushort ZZ = static_cast<ushort>(depth); // Z
        float Z = static_cast<float>(ZZ);
        float X = (c-cx)*depth/fx; // X
        float Y = (r-cy)*depth/fy; // Y
        // if(c==640 && r==360) std::cout <<"Depth of the central pixel: "<< depth<< " (mm)"<<std::endl;

        point tmp, tmp_base;
        tmp.x = Z / 1000.;
        tmp.y = -X / 1000.;
        tmp.z = -Y / 1000.;

        //tf camera_link -> base_link
        tmp_base.x = tf_matrix[0][0] * tmp.x + tf_matrix[0][1] * tmp.y + tf_matrix[0][2] * tmp.z + tf_matrix[0][3];
        tmp_base.y = tf_matrix[1][0] * tmp.x + tf_matrix[1][1] * tmp.y + tf_matrix[1][2] * tmp.z + tf_matrix[1][3];
        tmp_base.z = tf_matrix[2][0] * tmp.x + tf_matrix[2][1] * tmp.y + tf_matrix[2][2] * tmp.z + tf_matrix[2][3];
        pc_create.push_back(tmp_base);

        // -----> create 2D laser
        if (std::isnan(tmp_base.x) || std::isnan(tmp_base.y) || std::isnan(tmp_base.z)) continue;
        if (tmp_base.z > max_height_ || tmp_base.z < min_height_) continue;
        double range = hypot(tmp_base.x, tmp_base.y);
        if (range < range_min_) continue;
        if (range > range_max_) continue;
        double angle = atan2(tmp_base.y, tmp_base.x);
        if (angle < angle_min_ || angle > angle_max_) continue;
        //overwrite range at laserscan ray if new range is smaller
        int index = (angle - angle_min_) / angle_increment_;
        if (range < ls.ranges[index]) {
          ls.ranges[index] = range;
          laser_output.ranges[index] = range; //ROS
        }
        // <----- create 2D laser



        //ROS
        pcl::PointXYZRGB pt;
        pt.x = tmp_base.x ;
        pt.y = tmp_base.y ;
        pt.z = tmp_base.z ;
        cloud->push_back(pt); //ROS
    }
  }

  // ROS
  sensor_msgs::PointCloud2 ros_points;
  toROSMsg(*cloud, ros_points);
  ros_points.header.frame_id = "base_link";
  ros_points.header.stamp = ros::Time::now(); // time
  pub_pc.publish(ros_points);

  laser_output.angle_min = angle_min_;
  laser_output.angle_max = angle_max_;
  laser_output.angle_increment = angle_increment_;
  laser_output.time_increment = 0.0;
  laser_output.scan_time = scan_time_;
  laser_output.range_min = range_min_;
  laser_output.range_max = range_max_;
  laser_output.header.stamp = ros::Time::now(); // time
  laser_output.header.frame_id = "base_link";
  pub_laser.publish(laser_output);
  // ros

  // return pc_create;
  return ls;
}


cmd_vel DT_kv260_Node::obstacle_avoidance(laser ls){
  cmd_vel motor_command;

  float min_range = 99999;
	int min_range_angle = -120;

  volatile int j; //warning: iteration 241 invokes undefined behavior [-Waggressive-loop-optimizations] @@???
	for(j=0; j<241;j++) //increment by one degree
	{
  	if(ls.ranges[j]<min_range && ls.ranges[j]!=0){
			min_range = ls.ranges[j];
			min_range_angle = j-120;
    }
	}
	cout<<"minimum range is "<<min_range<<" at an angle of "<<min_range_angle<<endl;

	if(min_range<=0.5)  // min_range<=0.5 gave box pushing like behaviour, min_range<=1.2 gave obstacle avoidance
	{
		if(min_range_angle<0)
		{
			 motor_command.angular_z=0.25;
			 motor_command.linear_x=0;
			 printf("left\n");
		}
		else
		{
			 motor_command.angular_z=-0.25;
			 motor_command.linear_x=0;
			 printf("right\n");
		}
	}
	else
	{
		motor_command.linear_x=0.4;
		motor_command.angular_z=0;
		printf("straight\n");
	}

  return motor_command;
}


DT_kv260_Node::DT_kv260_Node():it_(nh_){
  sl_oc::tools::StereoSgbmPar stereoPar;
  std::string calibration_file;
  cv::Mat cameraMatrix_left, cameraMatrix_right;
  cv::Ptr<cv::StereoSGBM> left_matcher;
  cv::Mat map_left_x, map_left_y;
  cv::Mat map_right_x, map_right_y;
  uint64_t sn, serial_number;
  double baseline, fx, fy, cx, cy;


  // ros
  pub_left = it_.advertise("left_raw", 1);
  pub_pc = nh_.advertise<sensor_msgs::PointCloud2>("points", 1);
  pub_laser = nh_.advertise<sensor_msgs::LaserScan>("laser", 1);
  // ros

  // ----> Create Video Capture
  sl_oc::video::VideoParams params;
  #ifdef EMBEDDED_ARM
      params.res = sl_oc::video::RESOLUTION::VGA;
  #else
      params.res = sl_oc::video::RESOLUTION::HD720;
  #endif
  params.fps = sl_oc::video::FPS::FPS_30;
  sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::INFO;
  params.verbose = verbose;
  sl_oc::video::VideoCapture cap(params);
  if( !cap.initializeVideo() ){
      std::cerr << "Cannot open camera video capture" << std::endl;
      std::cerr << "See verbosity level for more details." << std::endl;
  }
  sn = cap.getSerialNumber();
  std::cout << "Connected to camera sn: " << sn << std::endl;
  // <---- Create Video Capture


  // ----> Retrieve calibration file from Stereolabs server
  // ZED Calibration
  serial_number = sn;
  // Download camera calibration file
  if( !sl_oc::tools::downloadCalibrationFile(serial_number, calibration_file) )
  {
      std::cerr << "Could not load calibration file from Stereolabs servers" << std::endl;
  }
  std::cout << "Calibration file found. Loading..." << std::endl;

  // ----> Frame size
  int w, h;
  cap.getFrameSize(w,h);
  // <---- Frame size

  // ----> Initialize calibration
  baseline=0;
  sl_oc::tools::initCalibration(calibration_file, cv::Size(w/2,h), map_left_x, map_left_y, map_right_x, map_right_y,
                                cameraMatrix_left, cameraMatrix_right, &baseline);

  fx = cameraMatrix_left.at<double>(0,0);
  fy = cameraMatrix_left.at<double>(1,1);
  cx = cameraMatrix_left.at<double>(0,2);
  cy = cameraMatrix_left.at<double>(1,2);

  std::cout << " Camera Matrix L: \n" << cameraMatrix_left << std::endl << std::endl;
  std::cout << " Camera Matrix R: \n" << cameraMatrix_right << std::endl << std::endl;




  //Note: you can use the tool 'zed_open_capture_depth_tune_stereo' to tune the parameters and save them to YAML
  if(!stereoPar.load())
  {
      stereoPar.save(); // Save default parameters.
  }

  left_matcher = cv::StereoSGBM::create(stereoPar.minDisparity,stereoPar.numDisparities,stereoPar.blockSize);
  left_matcher->setMinDisparity(stereoPar.minDisparity);
  left_matcher->setNumDisparities(stereoPar.numDisparities);
  left_matcher->setBlockSize(stereoPar.blockSize);
  left_matcher->setP1(stereoPar.P1);
  left_matcher->setP2(stereoPar.P2);
  left_matcher->setDisp12MaxDiff(stereoPar.disp12MaxDiff);
  left_matcher->setMode(stereoPar.mode);
  left_matcher->setPreFilterCap(stereoPar.preFilterCap);
  left_matcher->setUniquenessRatio(stereoPar.uniquenessRatio);
  left_matcher->setSpeckleWindowSize(stereoPar.speckleWindowSize);
  left_matcher->setSpeckleRange(stereoPar.speckleRange);

  stereoPar.print();
  // <---- Stereo matcher initialization

  cout<<"dt-kv260 one node initial done"<<endl;


  // Infinite video grabbing loop
  while (ros::ok())
  {
    // Get last available frame
    const sl_oc::video::Frame frame = cap.getLastFrame();

    // ----> If the frame is valid we can display it
    if(frame.data!=nullptr){
      cv::Mat left_disp_float = stereo_matching(frame, stereoPar, left_matcher,map_left_x, map_left_y,map_right_x, map_right_y);
      laser ls = create_depth_and_points_and_laser(left_disp_float, baseline, fx, fy, cx, cy);
      cmd_vel motor_command = obstacle_avoidance(ls);
    }
    // <---- If the frame is valid we can display it

    ros::spinOnce();
  }
}


int main(int argc, char **argv)
{
	init(argc, argv, "dt_kv260_one_node");
	DT_kv260_Node zrn;
	return 0;
}
