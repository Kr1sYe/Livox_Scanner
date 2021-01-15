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
#include <pcl/registration/icp.h>

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

#include <thread>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudTI;

class pointcloud_publisher
{
public:
    pointcloud_publisher(/* args */);
    ~pointcloud_publisher();
    void GetTF();
    void PubTotal();
    void GetLidarData(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void GetJointStatesData(const sensor_msgs::JointStatePtr& joint_msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subLivoxCloud_;
    ros::Subscriber subAuboJointStates_;
    ros::Publisher pubPointCloud_, pubPointCloud_Once_, pubPointCloud_Total_;

    tf::TransformListener tflistener_;
    tf::StampedTransform Livox_to_Odom_;

    std::mutex glb_pose_lock_;

    PointCloudTI::Ptr pc_totol_;
    PointCloudTI::Ptr pc_one_station_;
    // PointCloudTI::Ptr pc_cache_ptr_;

    bool inScanning_, inLastScanning_, inMoving_;
    Eigen::Matrix4d curr_tran_matrix_;

    std::string map_file_path;

    // thread
    std::thread *mthread_tf{nullptr};
    std::thread *mthread_pub_total{nullptr};
    // ros::AsyncSpinner *spinner;

};

pointcloud_publisher::pointcloud_publisher(/* args */) : pc_totol_(new PointCloudTI), pc_one_station_(new PointCloudTI), inScanning_(false), inLastScanning_(false), inMoving_(true)
{
    std::cout << "Construct Class" << std::endl;

    // spinner = new ros::AsyncSpinner(4);
    // spinner.start();

    subLivoxCloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 10, &pointcloud_publisher::GetLidarData, this);
    subAuboJointStates_ = nh_.subscribe("/joint_states",10, &pointcloud_publisher::GetJointStatesData, this);
    pubPointCloud_Once_ = nh_.advertise<sensor_msgs::PointCloud2>("once_lidar_pointcloud", 10);
    pubPointCloud_Total_ = nh_.advertise<sensor_msgs::PointCloud2>("total_lidar_pointcloud", 10);

    pc_one_station_->clear();

    // pc_totol_ = new PointCloudTI;
    // PointCloudTI::Ptr pc_cache_ptr(new PointCloudTI);
    ros::param::get("~map_file_path",map_file_path);

    // run();
    mthread_tf = new std::thread(&pointcloud_publisher::GetTF, this); 
    mthread_pub_total = new std::thread(&pointcloud_publisher::PubTotal, this);

}

pointcloud_publisher::~pointcloud_publisher()
{
    std::cout << "Destruct Class" << std::endl;
    
    //--------------------------save map pcd---------------
    std::string all_points_pcd_filename("/home/k/livox_ws/livox_all_points.pcd");

    // pcl::PointCloud<PointCloudTI> all_points;
    // all_points = *pc_totol_;
    if (pc_totol_->size() > 0 ) {
        pcl::PCDWriter pcd_writer;
        std::cout << "saving... pcd file in" << all_points_pcd_filename << std::endl;

        PointCloudTI::Ptr pc_totol_filted(new PointCloudTI);
        float leafSize = 0.01;
        // 执行降采样------------------------
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
        // 设置输入点云
        voxel.setInputCloud(pc_totol_);
        // 设置体素网格的大小
        voxel.setLeafSize(leafSize, leafSize, leafSize);
        // 执行滤波, 并将结果保存到cloud_filterd
        voxel.filter(*pc_totol_filted);

        pcd_writer.writeBinary(all_points_pcd_filename, *pc_totol_filted);
    } else {
        std::cout << "no points saved" << std::endl;;
    }

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
        float leafSize = 0.01;
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

void pointcloud_publisher::GetTF()
{
    std::cout << "GetTF" << std::endl;
    
    ros::Rate loop_sleep(50);
    while(ros::ok())
    {
        // std::cout << "[Debug] listening" << std::endl;
        // 获取Livox到Odom位姿
        try{
            tflistener_.lookupTransform("/odom", "/livox_link",
                                        ros::Time(0), Livox_to_Odom_);
        }
        catch (tf::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            // ros::Duration(1.0).sleep();
            continue;
        }

        // ROS_WARN("[Debug] Livox TF:=(%f,%f,%f), q(%f,%f,%f,%f)", 
        //             Livox_to_Odom_.getOrigin().x(),
        //             Livox_to_Odom_.getOrigin().y(),
        //             Livox_to_Odom_.getOrigin().z(),
        //             Livox_to_Odom_.getRotation().w(),
        //             Livox_to_Odom_.getRotation().x(),
        //             Livox_to_Odom_.getRotation().y(),
        //             Livox_to_Odom_.getRotation().z());

        // if(pc_totol_->size() != 0){
        //     sensor_msgs::PointCloud2 msg;
        //     PointCloudTI::Ptr pc_totol_filted_msg(new PointCloudTI);
        //     float leafSize = 0.1;
        //     // 执行降采样------------------------
        //     pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
        //     // 设置输入点云
        //     voxel.setInputCloud(pc_totol_);
        //     // 设置体素网格的大小
        //     voxel.setLeafSize(leafSize, leafSize, leafSize);
        //     // 执行滤波, 并将结果保存到cloud_filterd
        //     voxel.filter(*pc_totol_filted_msg);
        //     // pcl::toROSMsg(*pc_cache_ptr_,msg);
        //     pcl::toROSMsg(*pc_totol_filted_msg,msg);
            
        //     msg.header.frame_id = "odom";
        //     msg.header.stamp = ros::Time::now();
        //     pubPointCloud_Total_.publish(msg);
        // }

        loop_sleep.sleep();
        // ros::spinOnce();
    }
}

void pointcloud_publisher::GetLidarData(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    std::cout << "inMoving_ && !inScanning_: " << inMoving_ << ", " << inScanning_ << std::endl;
    if(inMoving_ && !inScanning_){
        if(pc_one_station_->size() != 0){
            
            // matching
            if(pc_totol_->size() != 0){
                ROS_WARN("matching !!!!!!!!!!!!!!!!!!!!!!!!!");
                pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;  //创建ICP的实例类
                pcl::PointCloud<pcl::PointXYZRGB> final_cloud;//配准后点云
                
                PointCloudTI::Ptr pc_one_station_down(new PointCloudTI);
                float leafSize = 0.05;
                // 执行降采样------------------------
                pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
                // 设置输入点云
                voxel.setInputCloud(pc_one_station_);
                // 设置体素网格的大小
                voxel.setLeafSize(leafSize, leafSize, leafSize);
                // 执行滤波, 并将结果保存到cloud_filterd
                voxel.filter(*pc_one_station_down);

                PointCloudTI::Ptr pc_totol_down(new PointCloudTI);
                // 执行降采样------------------------
                pcl::VoxelGrid<pcl::PointXYZRGB> voxel_total;
                // 设置输入点云
                voxel_total.setInputCloud(pc_totol_);
                // 设置体素网格的大小
                voxel_total.setLeafSize(leafSize, leafSize, leafSize);
                // 执行滤波, 并将结果保存到cloud_filterd
                voxel_total.filter(*pc_totol_down);

                icp.setInputSource(pc_one_station_down);
                icp.setInputTarget(pc_totol_down);
                icp.setMaxCorrespondenceDistance(0.2);  // 忽略在此距离之外的点，对配准影响较大
                icp.setTransformationEpsilon(1e-10);    // 上次转换与当前转换的差值
                icp.setEuclideanFitnessEpsilon(0.001);  // 前后两次迭代误差的差值 设置收敛条件是均方误差和小于阈值，停止迭代
                icp.setMaximumIterations(100);          // 最大迭代次数
                icp.align(final_cloud);

                // PointCloudTI::Ptr final_pc_one_station(new PointCloudTI); 
                // pcl::transformPointCloud(final_cloud,*final_pc_one_station,curr_tran_matrix_);

                // icp匹配后的转换矩阵及得分
                std::cout << "---------------------------------------------------------" << std::endl;
                std::cout << "has converged: " << icp.hasConverged() << std::endl
                          << "score: " << icp.getFitnessScore() << std::endl;
                std::cout << icp.getFinalTransformation() << std::endl; 

                // *pc_totol_ += *final_pc_one_station;
                *pc_totol_ += final_cloud;

            }
            // initialization
            else{
                *pc_totol_ += *pc_one_station_;
                std::cout << "initialization !!!" << std::endl;
            }

            pc_one_station_->clear();
        }
        return;
    }

    // if(pc_totol_->size() != 0){
    //     sensor_msgs::PointCloud2 msg;
    //     PointCloudTI::Ptr pc_totol_filted_msg(new PointCloudTI);
    //     float leafSize = 0.1;
    //     // 执行降采样------------------------
    //     pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    //     // 设置输入点云
    //     voxel.setInputCloud(pc_totol_);
    //     // 设置体素网格的大小
    //     voxel.setLeafSize(leafSize, leafSize, leafSize);
    //     // 执行滤波, 并将结果保存到cloud_filterd
    //     voxel.filter(*pc_totol_filted_msg);
    //     pcl::toROSMsg(*pc_totol_filted_msg,msg);
        
    //     msg.header.frame_id = "odom";
    //     msg.header.stamp = ros::Time::now();
    //     pubPointCloud_Total_.publish(msg);
    // }

    // std::cout << "[Debug] Processing Livox PointCloud" << std::endl;

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
            // temp_point.r = 0x0;
            // temp_point.g = green & 0xff;
            // temp_point.b = 0xff;
            temp_point.r = 0;
            temp_point.g = green;
            temp_point.b = 255;
        }
        else if (reflection_map < 90)
        {
            int blue = (((90 - reflection_map) * 255) / 60);
            // temp_point.r = 0x0;
            // temp_point.g = 0xff;
            // temp_point.b = blue & 0xff;
            temp_point.r = 0;
            temp_point.g = 255;
            temp_point.b = blue;
        }
        else if (reflection_map < 150)
        {
            int red = ((reflection_map - 90) * 255) / 60;
            // temp_point.r = red & 0xff;
            // temp_point.g = 0xff;
            // temp_point.b = 0x0;
            temp_point.r = red;
            temp_point.g = 255;
            temp_point.b = 0;
        }
        else
        {
            int green = ((255 - reflection_map) * 255) / (256 - 150);
            // temp_point.r = 0xff;
            // temp_point.g = green & 0xff;
            // temp_point.b = 0x0;
            temp_point.r = 255;
            temp_point.g = green;
            temp_point.b = 0;
        }
        pc_temp_ptr->push_back(temp_point);
    }

    Eigen::Matrix4d tran_matrix=tran_isometry.matrix();
    curr_tran_matrix_ = tran_matrix;
    // std::cout << "tran_matrix: \n" << tran_matrix << std::endl;
    
    // pc_one_station_->clear();
    PointCloudTI::Ptr pc_cache_ptr_(new PointCloudTI);  
    pcl::transformPointCloud(*pc_temp_ptr,*pc_cache_ptr_,tran_matrix);
    *pc_one_station_ += *pc_cache_ptr_;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*pc_cache_ptr_,msg);
    
    msg.header.frame_id = "odom";
    msg.header.stamp.sec = cloud_msg->header.stamp.sec;
    msg.header.stamp.nsec = cloud_msg->header.stamp.nsec;
    pubPointCloud_Once_.publish(msg);

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

void pointcloud_publisher::GetJointStatesData(const sensor_msgs::JointStatePtr& joint_msg)
{
    if(std::fabs(joint_msg->position[0]) > 1*3.1415926/180/1000){
        inScanning_ = true;
        std::cout << "SCAN !!!" << std::endl;
    }
    else{
        inScanning_ = false;
    }

    if (!inScanning_ && !inLastScanning_)
    {
        inMoving_ = true;
        ROS_WARN("MOVE !!!");
    }
    else
    {
        inMoving_ = false;
    }
    
    
        
    inLastScanning_ = inScanning_;

}

void pointcloud_publisher::PubTotal()
{
    ros::Rate loop_sleep(0.5);
    while(ros::ok())
    {
        std::cout << "[Debug] Pub Total" << std::endl;

        if(pc_totol_->size() != 0){
            sensor_msgs::PointCloud2 msg;
            PointCloudTI::Ptr pc_totol_filted_msg(new PointCloudTI);
            float leafSize = 0.1;
            // 执行降采样------------------------
            pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
            // 设置输入点云
            voxel.setInputCloud(pc_totol_);
            // 设置体素网格的大小
            voxel.setLeafSize(leafSize, leafSize, leafSize);
            // 执行滤波, 并将结果保存到cloud_filterd
            voxel.filter(*pc_totol_filted_msg);
            // pcl::toROSMsg(*pc_cache_ptr_,msg);
            pcl::toROSMsg(*pc_totol_filted_msg,msg);
            
            msg.header.frame_id = "odom";
            msg.header.stamp = ros::Time::now();
            pubPointCloud_Total_.publish(msg);
        }

        loop_sleep.sleep();
        // ros::spinOnce();
    }
}


#endif