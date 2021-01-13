#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <queue>
#include <deque>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <pthread.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <nav_msgs/Odometry.h>
// #include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;

bool save_points = false;


using namespace std;
// typedef pcl::PointXYZI PointTI;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudTI;
ros::Publisher pointcloud_pub, total_pointcloud_pub;

// typedef struct
// {
//     int32_t x;
//     int32_t y;
//     int32_t z;
//     uint8_t intensity;
// } __attribute__((packed)) lidar_point;

float glb_angle;
mutex glb_angle_lock;

// geometry_msgs::TransformStamped Livox_to_Odom;
tf::StampedTransform Livox_to_Odom;
mutex glb_pose_lock;

tf::TransformListener tflistener;
// void GetLivoxPose (const sensor_msgs::JointStateConstPtr& msgJointState, const nav_msgs::OdometryConstPtr& msgOdom) 
void GetLivoxPose (const nav_msgs::OdometryConstPtr& msgOdom) 
{
    // 获取Livox到Odom位姿
    try{
        // tf2_ros::Buffer tfBuffer;
        // tf2_ros::TransformListener tfListener(tfBuffer);
        // tfBuffer.canTransform("collision_virtual_ee_link", "obj_msg", ros::Time(0), ros::Duration(1.0));
        // Livox_to_Odom = tfBuffer.lookupTransform("odom", "livox_link", ros::Time(0));
        tflistener.lookupTransform("/odom", "/livox_link",
                                    ros::Time(0), Livox_to_Odom);
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        // continue;
        // return false;
    }

    ROS_WARN("[Debug] Livox TF:=(%f,%f,%f), q(%f,%f,%f,%f)", 
                Livox_to_Odom.getOrigin().x(),
                Livox_to_Odom.getOrigin().y(),
                Livox_to_Odom.getOrigin().z(),
                Livox_to_Odom.getRotation().w(),
                Livox_to_Odom.getRotation().x(),
                Livox_to_Odom.getRotation().y(),
                Livox_to_Odom.getRotation().z());
}

// void GetJointData(const sensor_msgs::JointStateConstPtr& joint_msg)
// {
//     glb_angle_lock.lock();
//     glb_angle = joint_msg->position[0];
//     glb_angle_lock.unlock();
// }

PointCloudTI::Ptr pc_totol(new PointCloudTI);
PointCloudTI::Ptr pc_cache_ptr(new PointCloudTI);
void GetLidarData(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

    pcl::PointCloud<pcl::PointXYZI> lidarPoints;
    pcl::fromROSMsg(*cloud_msg,lidarPoints);
    int dnum=lidarPoints.points.size();
    
    glb_pose_lock.lock();
    Eigen::Isometry3d tran_isometry = Eigen::Isometry3d::Identity();
    tf::Matrix3x3 tf_rotation;
    tf::Transform tf_transform;
    tf_transform.setRotation(tf::Quaternion(Livox_to_Odom.getRotation().x(),
                                            Livox_to_Odom.getRotation().y(),
                                            Livox_to_Odom.getRotation().z(),
                                            Livox_to_Odom.getRotation().w()));
    tf_transform.setOrigin(tf::Vector3(Livox_to_Odom.getOrigin().x(), 
                                       Livox_to_Odom.getOrigin().y(), 
                                       Livox_to_Odom.getOrigin().z()));

    tf::transformTFToEigen(tf_transform, tran_isometry);

    glb_pose_lock.unlock();

    PointCloudTI::Ptr pc_temp_ptr(new PointCloudTI);    
    for (int i = 0; i < dnum; i++)
    {
        pcl::PointXYZRGB temp_point;
        temp_point.x = lidarPoints.points[i].x;
        temp_point.y = lidarPoints.points[i].y;
        temp_point.z = lidarPoints.points[i].z;
        int reflection_map = (int)lidarPoints.points[i].intensity;
        if (reflection_map < 30)
        {
            int green = (reflection_map * 255 / 30);
            temp_point.r = 0x0;
            temp_point.g = green & 0xff;
            temp_point.b = 0xff;
        }
        else if (reflection_map < 90)
        {
            int blue = (((90 - reflection_map) * 255) / 60);
            temp_point.r = 0x0;
            temp_point.g = 0xff;
            temp_point.b = blue & 0xff;
        }
        else if (reflection_map < 150)
        {
            int red = ((reflection_map - 90) * 255) / 60;
            temp_point.r = red & 0xff;
            temp_point.g = 0xff;
            temp_point.b = 0x0;
        }
        else
        {
            int green = ((255 - reflection_map) * 255) / (256 - 150);
            temp_point.r = 0xff;
            temp_point.g = green & 0xff;
            temp_point.b = 0x0;
        }
        pc_temp_ptr->push_back(temp_point);
    }

    // Eigen::Isometry3f tran_isometry = Eigen::Isometry3f::Identity();
    // Eigen::Matrix3f rotate_matrix;
    // Eigen::Matrix4f tran_matrix;

    // Eigen::AngleAxisf rotate_angle_axis(rotate_angle, Eigen::Vector3f(0, 0, 1));
    // rotate_matrix = rotate_angle_axis.toRotationMatrix();
    // tran_isometry.rotate(rotate_matrix);
    Eigen::Matrix4d tran_matrix=tran_isometry.matrix();
    
    pc_cache_ptr->clear();
    pcl::transformPointCloud(*pc_temp_ptr,*pc_cache_ptr,tran_matrix);
    *pc_totol += *pc_cache_ptr;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*pc_cache_ptr,msg);

    //save points
    if(save_points){
        string save_name = "./pointscloud2.csv";
        ofstream outfile;
        outfile.open(save_name,ios::app);
        for(int pid=0;pid<dnum;pid++){
            outfile<<pc_cache_ptr->points[pid].x<<','
                <<pc_cache_ptr->points[pid].y<<','
                <<pc_cache_ptr->points[pid].z<<','
                <<lidarPoints.points[pid].intensity
                <<"\n";
        }
        outfile.close();
    }
    msg.header.frame_id = "livox_link";
    msg.header.stamp.sec = cloud_msg->header.stamp.sec;
    msg.header.stamp.nsec = cloud_msg->header.stamp.nsec;
    pointcloud_pub.publish(msg);
    // total_pointcloud_pub.publish();

    ROS_INFO("Livox TF:=(%f,%f,%f), q(%f,%f,%f,%f) topic_num:= %d", 
                Livox_to_Odom.getOrigin().x(),
                Livox_to_Odom.getOrigin().y(),
                Livox_to_Odom.getOrigin().z(),
                Livox_to_Odom.getRotation().w(),
                Livox_to_Odom.getRotation().x(),
                Livox_to_Odom.getRotation().y(),
                Livox_to_Odom.getRotation().z(),
                dnum);

}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "pointcloud_publisher");
    
    ros::NodeHandle nh;
    ros::Subscriber customCloud = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar",10,GetLidarData);

    ros::Subscriber odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom",10,GetLivoxPose);

    // message_filters::Subscriber<sensor_msgs::JointState> joint_sub_(nh, "/joint_states", 1); 
    // message_filters::Subscriber<nav_msgs::Odometry> odom_sub_(nh, "/odom", 1);
    // message_filters::TimeSynchronizer<sensor_msgs::JointState, nav_msgs::Odometry> sync(joint_sub_, odom_sub_, 10);
    // sync.registerCallback(boost::bind(&GetLivoxPose, _1, _2));

    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("scaner_lidar_pointcloud", 10);
    ros::spin();
    return 0;
}
