// @Time : 2021/8/12 下午9:30
// @Author : WenkyJong
// @Site : MianYang SWUST
// @File : create_map_node.cpp
// @Contact: wenkyjong1996@gmail.com
// @desc:

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

void ImageHandler(const sensor_msgs::ImageConstPtr &img_msg);
void DepthHandler(const sensor_msgs::ImageConstPtr &img_msg);

int main(int argc ,char **argv){
    ros::init(argc, argv, "map_create_");
    ros::NodeHandle n;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    std::string image_topic{"/camera/color/image_raw"};
    std::string depth_topic{"/camera/aligned_depth_to_color/image_raw"};
    n.getParam("",image_topic);
    n.getParam("",depth_topic);

    auto subImage = n.subscribe(image_topic, 100, ImageHandler);
    auto subDepth = n.subscribe(depth_topic,100,DepthHandler);

    ros::spin();
    return 0;
}

