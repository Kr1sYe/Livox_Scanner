#include <ros/ros.h>
#include <iostream> 
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>      //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h>    //PCL对各种格式的点的支持头文件
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

int main(int argc, char** argv)
{
    std::cout << "Test ICP" << std::endl;

    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub_station_1 = nh.advertise<sensor_msgs::PointCloud2> ("pcl_station_1", 1);
    ros::Publisher pcl_pub_station_2 = nh.advertise<sensor_msgs::PointCloud2> ("pcl_station_2", 1);
    ros::Publisher pcl_pub_station_2_aligned = nh.advertise<sensor_msgs::PointCloud2> ("aligned_pcl_station_2", 1);
    // ros::Publisher pcl_pub_station_2_test = nh.advertise<sensor_msgs::PointCloud2> ("test_pcl_station_2", 1);

    // 定义发布的消息
    sensor_msgs::PointCloud2 output_1;
    sensor_msgs::PointCloud2 output_2;
    sensor_msgs::PointCloud2 output_2_aligned;
    // sensor_msgs::PointCloud2 output_2_test;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_station_1 (new pcl::PointCloud<pcl::PointXYZRGB>); // 创建点云（指针）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_station_2 (new pcl::PointCloud<pcl::PointXYZRGB>); // 创建点云（指针）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_station_1_down(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_station_2_down(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud_station_1) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
    {
        PCL_ERROR ("Couldn't read file \n"); //文件不存在时，返回错误，终止程序。
        return (-1);
    }
    std::cout << "Loaded "
                << cloud_station_1->width * cloud_station_1->height
                << " data points from pcd file with the following fields: "
                << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[2], *cloud_station_2) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
    {
        PCL_ERROR ("Couldn't read file \n"); //文件不存在时，返回错误，终止程序。
        return (-1);
    }
    std::cout << "Loaded "
                << cloud_station_2->width * cloud_station_2->height
                << " data points from pcd file with the following fields: "
                << std::endl;

    float leafSize = 0.05;
    // 执行降采样------------------------
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_1;
    // 设置输入点云
    voxel_1.setInputCloud(cloud_station_1);
    // 设置体素网格的大小
    voxel_1.setLeafSize(leafSize, leafSize, leafSize);
    // 执行滤波, 并将结果保存到cloud_filterd
    voxel_1.filter(*cloud_station_1_down);

    // 执行降采样------------------------
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_2;
    // 设置输入点云
    voxel_2.setInputCloud(cloud_station_2);
    // 设置体素网格的大小
    voxel_2.setLeafSize(leafSize, leafSize, leafSize);
    // 执行滤波, 并将结果保存到cloud_filterd
    voxel_2.filter(*cloud_station_2_down);

    //Convert the cloud to ROS message
    pcl::toROSMsg(*cloud_station_1_down, output_1);
    output_1.header.frame_id = "odom";
    std::cout << "Filterd size: " << output_1.data.size() << std::endl;

    //Convert the cloud to ROS message
    pcl::toROSMsg(*cloud_station_2_down, output_2);
    output_2.header.frame_id = "odom";
    std::cout << "Filterd size: " << output_2.data.size() << std::endl;

    // -------------------------------------------------------------------------------------------------
    // [debug test -------------------------------------------------------------------------------------
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_station_2(new pcl::PointCloud<pcl::PointXYZRGB>);  
    // Eigen::Isometry3d tran_isometry = Eigen::Isometry3d::Identity();
    // Eigen::Matrix3d rotate_matrix;

    // Eigen::AngleAxisd rotate_angle_axis(0.1, Eigen::Vector3d(0, 0, 1));
    // rotate_matrix = rotate_angle_axis.toRotationMatrix();
    // tran_isometry.rotate(rotate_matrix);
    // Eigen::Matrix4d tran_matrix=tran_isometry.matrix();
    // std::cout << "rotation matrix inv: \n" << tran_matrix.inverse() << std::endl;
    // pcl::transformPointCloud(*cloud_station_2_down,*test_station_2,tran_matrix);

    // //Convert the cloud to ROS message
    // pcl::toROSMsg(*test_station_2, output_2_test);
    // output_2_test.header.frame_id = "odom";
    // std::cout << "Filterd size: " << output_2_test.data.size() << std::endl;
    // -------------------------------------------------------------------------------------------------
    // -------------------------------------------------------------------------------------------------]

    // ICP
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;  //创建ICP的实例类
    pcl::PointCloud<pcl::PointXYZRGB> final_station_2;//配准后点云

    icp.setInputSource(cloud_station_2_down);
    // icp.setInputSource(test_station_2);

    icp.setInputTarget(cloud_station_1_down);
    // icp.setInputTarget(cloud_station_2_down);
    icp.setMaxCorrespondenceDistance(0.2);  // 忽略在此距离之外的点，对配准影响较大
    icp.setTransformationEpsilon(1e-10);    // 上次转换与当前转换的差值
    icp.setEuclideanFitnessEpsilon(0.001);  // 前后两次迭代误差的差值 设置收敛条件是均方误差和小于阈值，停止迭代
    icp.setMaximumIterations(100);          // 最大迭代次数
    icp.align(final_station_2);

    // icp匹配后的转换矩阵及得分
    std::cout << "---------------------------------------------------------" << std::endl;
    std::cout << "has converged: " << icp.hasConverged() << std::endl
                << "score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl; 

    //Convert the cloud to ROS message
    pcl::toROSMsg(final_station_2, output_2_aligned);
    output_2_aligned.header.frame_id = "odom";
    std::cout << "Filterd size: " << output_2_aligned.data.size() << std::endl;

    ros::Rate loop_rate(5);
    while (ros::ok())
    {
        pcl_pub_station_1.publish(output_1);
        pcl_pub_station_2.publish(output_2);
        pcl_pub_station_2_aligned.publish(output_2_aligned);
        // pcl_pub_station_2_test.publish(output_2_test);

        ros::spinOnce();
        loop_rate.sleep();

        // ROS_INFO("Publishing PCD PointCloud");
    }

    ros::shutdown();
}