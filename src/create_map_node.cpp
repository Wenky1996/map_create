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
#include <pcl/common/transforms.h>

#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr map(new pcl::PointCloud<pcl::PointXYZRGB>());

std::queue<nav_msgs::OdometryPtr> pose_vio_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointcloud_buf;

void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudmsg);
void KeyPoseHandler(const visualization_msgs::MarkerArray::ConstPtr &vioKeyPose);
void CreatMap();


Eigen::Isometry3d NavMsg2Isometry3D(nav_msgs::Odometry::Ptr NavMsg);

ros::Publisher map_pub;


int main(int argc ,char **argv){
    ros::init(argc, argv, "map_create");
    ros::NodeHandle n;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ros::Subscriber subPointCloud = n.subscribe<sensor_msgs::PointCloud2>("/fusion_pointcloud", 100, PointCloudHandler);
    ros::Subscriber subkeyPose = n.subscribe("key_odometrys",2000,KeyPoseHandler);
    map_pub = n.advertise<sensor_msgs::PointCloud2>("create_map", 100);
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
        sensor_msgs::PointCloud2ConstPtr pointCloudMsg;
        vioPose=pose_vio_buf.front();
        auto vio_stamp=vioPose->header.stamp.toSec();
        pointCloudMsg = pointcloud_buf.front();
        int i_cnt=0;
        auto pointCloud_stamp = pointCloudMsg->header.stamp;
        if(abs(vio_stamp-pointCloud_stamp.toSec())<0.03){//1/30小于1帧
            pose_vio_buf.pop();
            pointcloud_buf.pop();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(*pointCloudMsg,*pointCloud);
            Eigen::Isometry3d transform_pose = Eigen::Isometry3d::Identity();
            transform_pose = NavMsg2Isometry3D(vioPose);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_point(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::transformPointCloud(*pointCloud,*transformed_point,transform_pose.cast<float>());
            *map+=*transformed_point;
            if(i_cnt%30==0){
                sensor_msgs::PointCloud2 pointMapMsg;
                pcl::toROSMsg(*map,pointMapMsg);
                pointMapMsg.header.stamp=pointCloud_stamp;
                pointMapMsg.header.frame_id="map";
                map_pub.publish(pointMapMsg);
            }
        } else{
            if(vio_stamp>pointCloud_stamp.toSec()){
                pointcloud_buf.pop();
            } else{
                pose_vio_buf.pop();
            }
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}
/*
 *    transform NavMsg to Eigen Isometry3d
 */
Eigen::Isometry3d NavMsg2Isometry3D(nav_msgs::Odometry::Ptr NavMsg){
    Eigen::Quaterniond pose_quaternion(1,0,0,0);
    Eigen::Vector3d pose_transalte(0,0,0);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    pose_quaternion.w()=NavMsg->pose.pose.orientation.w;
    pose_quaternion.x()=NavMsg->pose.pose.orientation.x;
    pose_quaternion.y()=NavMsg->pose.pose.orientation.y;
    pose_quaternion.z()=NavMsg->pose.pose.orientation.z;
    pose_transalte<<NavMsg->pose.pose.position.x,NavMsg->pose.pose.position.y,NavMsg->pose.pose.position.z;
    T.rotate(pose_quaternion);
    T.pretranslate(pose_transalte);//Applies on the left the translation matri
                                   // x represented by the vector other to *this and returns a reference to
    return  T;
}
