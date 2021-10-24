// @Time : 2021/8/12 下午9:30
// @Author : WenkyJong
// @Site : MianYang SWUST
// @File : create_map_node.cpp
// @Contact: wenkyjong1996@gmail.com
// @desc:

#include <queue>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr map;


std::queue<nav_msgs::OdometryPtr> pose_vio_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointcloud_buf;

void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudmsg);
void KeyPoseHandler(const nav_msgs::Odometry::ConstPtr &vioKeyPose);
void CreatMap();

int main(int argc ,char **argv){
    ros::init(argc, argv, "map_create");
    ros::NodeHandle n;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ros::Subscriber subPointCloud = n.subscribe<sensor_msgs::PointCloud2>("/fusion_pointcloud", 100, PointCloudHandler);
    ros::Subscriber subkeyPose = n.subscribe("key_odometrys",2000,KeyPoseHandler);
    std::thread creat_map_thread{CreatMap};
    ros::spin();
    return 0;
}

void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudmsg) {
    pointcloud_buf.push(pointCloudmsg);
}

void KeyPoseHandler(const visualization_msgs::MarkerArray::ConstPtr &vioKeyPose){
    nav_msgs::Odometry::Ptr vioOdometry;
    vioOdometry->pose.pose.position.x = vioKeyPose->markers.front().pose.position.x;
    vioOdometry->pose.pose.position.y = vioKeyPose->markers.front().pose.position.y;
    vioOdometry->pose.pose.position.z = vioKeyPose->markers.front().pose.position.z;
    vioOdometry->pose.pose.orientation.z = vioKeyPose->markers.front().pose.orientation.z;
    vioOdometry->pose.pose.orientation.y=vioKeyPose->markers.front().pose.orientation.y;
    vioOdometry->pose.pose.orientation.x=vioKeyPose->markers.front().pose.orientation.x;
    vioOdometry->pose.pose.orientation.w=vioKeyPose->markers.front().pose.orientation.w;
    vioOdometry->header.stamp=vioKeyPose->markers.front().header.stamp;
    pose_vio_buf.push(vioOdometry);

}

void CreatMap(){
    nav_msgs::Odometry::Ptr vioPose;
    while(true){
        sensor_msgs::PointCloud2ConstPtr pointCloud;
        vioPose=pose_vio_buf.front();
        auto vio_stamp=vioPose->header.stamp.toSec();
        pointCloud = pointcloud_buf.front();
        auto pointCloud_stamp = pointCloud->header.stamp.toSec();
        if(abs(vio_stamp-pointCloud_stamp)<0.03){//1/30小于1帧
            pose_vio_buf.pop();
            pointcloud_buf.pop();
            Eigen::Isometry3d transform_pose = Eigen::Isometry3d::Identity();
            transform_pose.translate(vioPose->pose.pose.position);

            
        } else{
            if(vio_stamp>pointCloud_stamp){
                pointcloud_buf.pop();
            } else{
                pose_vio_buf.pop();
            }
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

Eigen::Isometry3d NavMsg2Isometry3D(sensor_msgs::PointCloud2ConstPtr NavMsg){
    Eigen::Quaterniond pose_quaternion(1,0,0,0);
    Eigen::Vector3d pose_transalte

}
