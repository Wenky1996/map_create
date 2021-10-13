// @Time : 2021/8/12 下午9:30
// @Author : WenkyJong
// @Site : MianYang SWUST
// @File : create_map_node.cpp
// @Contact: wenkyjong1996@gmail.com
// @desc:

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <message_filters/subscriber.h>



void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr & pointCloudmsg) ;
void KeyPoseHandler(const nav_msgs::Odometry::ConstPtr &vioKeyPose);


int main(int argc ,char **argv){
    ros::init(argc, argv, "map_create");
    ros::NodeHandle n;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::Subscriber subPointCloud = n.subscribe<sensor_msgs::PointCloud2>("/fusion_pointcloud", 100, PointCloudHandler);
    ros::Subscriber subkeyPose = n.subscribe("key_odometrys",2000,KeyPoseHandler);

    ros::spin();
    return 0;
}

void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr & pointCloudmsg) {

}

void KeyPoseHandler(const nav_msgs::Odometry::ConstPtr &vioKeyPose){

}

