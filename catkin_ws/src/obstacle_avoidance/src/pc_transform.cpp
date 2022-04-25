#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

// #include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

using namespace ros;
using namespace std;
using namespace pcl;

class PCTransfer
{
private:
    VoxelGrid<PointXYZ> voxel;

    tf::TransformListener listener;
    tf::StampedTransform tf_pose;
    Eigen::Matrix4f pose_matrix;
    Eigen::Matrix4f camera_init_to_base;

    float leaf_size = 0.01;
    sensor_msgs::PointCloud2 ros_base;
    PointCloud<PointXYZ>::Ptr pc;
    PointCloud<PointXYZ>::Ptr base;

    Publisher pub_base;
    Subscriber sub_base;

public:
    void pc_cb(const sensor_msgs::PointCloud2 msg);
    PCTransfer(NodeHandle &nh);
    ~PCTransfer();
};

PCTransfer::PCTransfer(NodeHandle &nh)
{
    param::get("~voxel_size", leaf_size);
    ROS_INFO("filter voxel size %f", leaf_size);

    ros_base.header.frame_id = "base_link";
    pc.reset(new PointCloud<PointXYZ>);
    base.reset(new PointCloud<PointXYZ>);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);


    pub_base = nh.advertise<sensor_msgs::PointCloud2>("points_base_link", 1);
    sub_base = nh.subscribe("points", 1, &PCTransfer::pc_cb, this);
    ROS_INFO("PCTransfer initialized");
}

PCTransfer::~PCTransfer()
{
}

void PCTransfer::pc_cb(const sensor_msgs::PointCloud2 msg)
{
    fromROSMsg(msg, *pc);
    try
    {
        ros::Duration five_seconds(5.0);
        listener.waitForTransform("base_link", "zed_mini", ros::Time(0), five_seconds);
        listener.lookupTransform("base_link", "zed_mini", ros::Time(0), tf_pose);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    pcl_ros::transformAsMatrix(tf_pose, pose_matrix);
    voxel.setInputCloud(pc);
    voxel.filter(*pc);
    pcl::transformPointCloud(*pc, *pc, pose_matrix);

    toROSMsg(*pc, ros_base);
    ros_base.header.frame_id = "base_link";
    ros_base.header.stamp = msg.header.stamp;
    pub_base.publish(ros_base);

}

int main(int argc, char **argv)
{
    init(argc, argv, "PCTransfer");
    NodeHandle nh;

    PCTransfer PCTransfer(nh);

    spin();

    return 0;
}
