// @Time : 2021/8/23 上午9:39
// @Author : WenkyJong
// @Site : MianYang SWUST
// @File : Rgbd2PointCloud_node.cpp
// @Contact: wenkyjong1996@gmail.com
// @desc:
#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>



using std::queue;
using std::string;


void ImageHandler(const sensor_msgs::ImageConstPtr &msg_image);
void DepthHandler(const sensor_msgs::ImageConstPtr &msg_depth);

int main(int argc ,char **argv){
    ros::init(argc, argv, "rgbd2pointcloud");
    ros::NodeHandle n;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    std::string image_topic{"/camera/color/image_raw"};
    std::string depth_topic{"/camera/aligned_depth_to_color/image_raw"};
    n.getParam("",image_topic);
    n.getParam("",depth_topic);

    auto subImage = n.subscribe("/camera/color/image_raw", 100, ImageHandler);
    auto subDepth = n.subscribe("/camera/aligned_depth_to_color/image_raw",100,DepthHandler);

    ros::spin();
    return 0;
}

void ImageHandler(const sensor_msgs::ImageConstPtr &msg_image){

}

void DepthHandler(const sensor_msgs::ImageConstPtr &msg_depth){

}