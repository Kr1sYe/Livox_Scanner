#ifndef POINTCLOUD_PUBLISHER_H_
#define POINTCLOUD_PUBLISHER_H_

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
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <queue>
#include <deque>
#include <mutex>
#include <chrono>

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

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudTI;

class pointcloud_publisher
{
public:
    pointcloud_publisher(/* args */);
    ~pointcloud_publisher();
    void run();
    void GetLidarData(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subLivoxCloud_;
    ros::Publisher pubPointCloud_, pubPointCloud_Total;

    tf::TransformListener tflistener_;
    tf::StampedTransform Livox_to_Odom_;

    std::mutex glb_pose_lock_;

    PointCloudTI::Ptr pc_totol_;
    // PointCloudTI::Ptr pc_cache_ptr_;

    std::string map_file_path;

};

pointcloud_publisher::pointcloud_publisher(/* args */) : pc_totol_(new PointCloudTI)
{
    std::cout << "Construct Class" << std::endl;
    subLivoxCloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 10, &pointcloud_publisher::GetLidarData, this);
    pubPointCloud_Total = nh_.advertise<sensor_msgs::PointCloud2>("scaner_lidar_pointcloud", 10);

    // pc_totol_ = new PointCloudTI;
    // PointCloudTI::Ptr pc_cache_ptr(new PointCloudTI);
    ros::param::get("~map_file_path",map_file_path);

    run();

    //--------------------------save map pcd---------------
    // std::string all_points_pcd_filename("/home/k/livox_ws/livox_all_points.pcd");

    // // pcl::PointCloud<PointCloudTI> all_points;
    // // all_points = *pc_totol_;
    // if (pc_totol_->size() > 0 ) {
    //     pcl::PCDWriter pcd_writer;
    //     std::cout << "saving... pcd file in" << all_points_pcd_filename << std::endl;
    //     pcd_writer.writeBinary(all_points_pcd_filename, *pc_totol_);
    // } else {
    //     std::cout << "no points saved" << std::endl;;
    // }

    //--------------------------save map xyz---------------
    std::string all_points_xyz_filename("/home/k/livox_ws/livox_all_points.xyz");
    if (pc_totol_->size() > 0 ) {
        std::ofstream ofs;
        ofs.open(all_points_xyz_filename, std::ios::out);
        if (!ofs.is_open())
        {
            std::cout << "open xyz file failed !!!" << std::endl;
            return;
        }

        std::cout << "saving... xyz file in " << all_points_xyz_filename << std::endl;
        PointCloudTI::Ptr pc_totol_filted(new PointCloudTI);
        float leafSize = 0.05;
        // 执行降采样------------------------
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
        // 设置输入点云
        voxel.setInputCloud(pc_totol_);
        // 设置体素网格的大小
        voxel.setLeafSize(leafSize, leafSize, leafSize);
        // 执行滤波, 并将结果保存到cloud_filterd
        voxel.filter(*pc_totol_filted);

        std::cout << "before down sample size: " << pc_totol_->size() << ", after down sample size: " << pc_totol_filted->size() << std::endl;
        for (size_t i = 0; i < pc_totol_filted->size(); i++)
        {
            ofs << pc_totol_filted->points[i].x << " "
                << pc_totol_filted->points[i].y << " "
                << pc_totol_filted->points[i].z << " "
                << pc_totol_filted->points[i].r << " "
                << pc_totol_filted->points[i].g << " "
                << pc_totol_filted->points[i].b << " "
                << std::endl;

        }
        ofs.close();

    } else {
        std::cout << "no points saved" << std::endl;;
    }
}

pointcloud_publisher::~pointcloud_publisher()
{
    std::cout << "Destruct Class" << std::endl;
}

void pointcloud_publisher::run()
{
    // std::cout << "listening" << std::endl;
    
    ros::Rate loop_sleep(10);
    while(ros::ok())
    {
        std::cout << "listening" << std::endl;
        // 获取Livox到Odom位姿
        try{
            tflistener_.lookupTransform("/odom", "/livox_link",
                                        ros::Time(0), Livox_to_Odom_);
        }
        catch (tf::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        ROS_WARN("[Debug] Livox TF:=(%f,%f,%f), q(%f,%f,%f,%f)", 
                    Livox_to_Odom_.getOrigin().x(),
                    Livox_to_Odom_.getOrigin().y(),
                    Livox_to_Odom_.getOrigin().z(),
                    Livox_to_Odom_.getRotation().w(),
                    Livox_to_Odom_.getRotation().x(),
                    Livox_to_Odom_.getRotation().y(),
                    Livox_to_Odom_.getRotation().z());

        std::cout << "test hex 0xff: " << 0xff << std::endl;

        loop_sleep.sleep();
        ros::spinOnce();
    }
}

void pointcloud_publisher::GetLidarData(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    std::cout << "Recive Livox PointCloud" << std::endl;

    pcl::PointCloud<pcl::PointXYZI> lidarPoints;
    pcl::fromROSMsg(*cloud_msg,lidarPoints);
    int dnum=lidarPoints.points.size();

    // glb_pose_lock_.lock();
    Eigen::Isometry3d tran_isometry = Eigen::Isometry3d::Identity();
    tf::Matrix3x3 tf_rotation;
    tf::Transform tf_transform;
    tf_transform.setRotation(tf::Quaternion(Livox_to_Odom_.getRotation().x(),
                                            Livox_to_Odom_.getRotation().y(),
                                            Livox_to_Odom_.getRotation().z(),
                                            Livox_to_Odom_.getRotation().w()));
    tf_transform.setOrigin(tf::Vector3(Livox_to_Odom_.getOrigin().x(), 
                                       Livox_to_Odom_.getOrigin().y(), 
                                       Livox_to_Odom_.getOrigin().z()));

    tf::transformTFToEigen(tf_transform, tran_isometry);
    // glb_pose_lock_.unlock();

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

    Eigen::Matrix4d tran_matrix=tran_isometry.matrix();
    // std::cout << "tran_matrix: \n" << tran_matrix << std::endl;
    
    // pc_cache_ptr_->clear();
    PointCloudTI::Ptr pc_cache_ptr_(new PointCloudTI);  
    pcl::transformPointCloud(*pc_temp_ptr,*pc_cache_ptr_,tran_matrix);
    *pc_totol_ += *pc_cache_ptr_;

    sensor_msgs::PointCloud2 msg;
    // pcl::toROSMsg(*pc_cache_ptr_,msg);
    pcl::toROSMsg(*pc_cache_ptr_,msg);

    msg.header.frame_id = "odom";
    msg.header.stamp.sec = cloud_msg->header.stamp.sec;
    msg.header.stamp.nsec = cloud_msg->header.stamp.nsec;
    pubPointCloud_Total.publish(msg);

    // save points
    // if(true){
    //     string save_name = "./pointscloud2.csv";
    //     ofstream outfile;
    //     outfile.open(save_name,ios::app);
    //     for(int pid=0;pid<dnum;pid++){
    //         outfile<<pc_cache_ptr->points[pid].x<<','
    //             <<pc_cache_ptr->points[pid].y<<','
    //             <<pc_cache_ptr->points[pid].z<<','
    //             <<lidarPoints.points[pid].intensity
    //             <<"\n";
    //     }
    //     outfile.close();
    // }

}


#endif