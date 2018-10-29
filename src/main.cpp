#include "stdafx.h"

#include <pcl/io/pcd_io.h>
#include <sstream>
#include <pcl/common/time.h>

using namespace global_param;

void
cloud_cb_16l (const sensor_msgs::PointCloud2ConstPtr& input) {

    double  t1=pcl::getTime();
    sensor_msgs::PointCloud2 output;

    //transfer msg to PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_data(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud_data);
    // Do data processing here...
    auto time_now=pcl::getTime();
    cloud_data->width = uint32_t(R_FANS_LINE16);
    cloud_data->height = uint32_t(cloud_data->size() / R_FANS_LINE16);
    if (cloud_data->size() % R_FANS_LINE16 != 0) {
        std::stringstream err_str;
        err_str << "the line of rfans is not" << R_FANS_LINE16;
        std::cout<<err_str.str()<<std::endl;
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_swap(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<PointSrc>::Ptr cloud_src(new pcl::PointCloud<PointSrc>);
    for(int idx_sweep=0;idx_sweep<cloud_data->height;idx_sweep++)
        for(int idx_beam=0;idx_beam<cloud_data->width;idx_beam++) {
            int idx = get_idx_with_offset(idx_beam, idx_sweep, cloud_data->height);
            pcl::PointXYZI temp_pt = cloud_data->points[idx];
            float x = temp_pt.x;
            float y = temp_pt.y;
            float z = temp_pt.z;
            float radius = sqrt(x * x + y * y + z * z);
            float angle = float(atan2(y, x) / M_PI * 180);
            PointSrc tmp(radius, angle);
            cloud_src->push_back(tmp);
            cloud_swap->push_back(temp_pt);
        }
    cloud_swap.swap(cloud_data);

    //TODO:cloud treat

    std::cout<<"total time is "<<pcl::getTime()-t1<<std::endl;
    
}

void
cloud_cb_32l (const sensor_msgs::PointCloud2ConstPtr& input) {

    double  t1=pcl::getTime();
    sensor_msgs::PointCloud2 output;

    //transfer msg to PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_data(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud_data);
    // Do data processing here...
    auto time_now=pcl::getTime();
    cloud_data->width = uint32_t(R_FANS_LINE32);
    cloud_data->height = uint32_t(cloud_data->size() / R_FANS_LINE32);
    if (cloud_data->size() % R_FANS_LINE32 != 0) {
        std::stringstream err_str;
        err_str << "the line of rfans is not" << R_FANS_LINE32;
        std::cout<<err_str.str()<<std::endl;
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_swap(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<PointSrc>::Ptr cloud_src(new pcl::PointCloud<PointSrc>);
    for(int idx_sweep=0;idx_sweep<cloud_data->height;idx_sweep++)
        for(int idx_beam=0;idx_beam<cloud_data->width;idx_beam++) {
            int idx = get_idx_with_offset(idx_beam, idx_sweep, cloud_data->height);
            pcl::PointXYZI temp_pt = cloud_data->points[idx];
            float x = temp_pt.x;
            float y = temp_pt.y;
            float z = temp_pt.z;
            float radius = sqrt(x * x + y * y + z * z);
            float angle = float(atan2(y, x) / M_PI * 180);
            PointSrc tmp(radius, angle);
            cloud_src->push_back(tmp);
            cloud_swap->push_back(temp_pt);
        }
    cloud_swap.swap(cloud_data);

    //TODO:cloud treat

    std::cout<<"total time is "<<pcl::getTime()-t1<<std::endl;
    
}

int
main (int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "rfans_merge_node");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub1 = nh.subscribe("/ns1/rfans_driver/rfans_points", 1, cloud_cb_16l);
    ros::Subscriber sub2 = nh.subscribe("/ns2/rfans_driver/rfans_points", 1, cloud_cb_32l);

    // ros::Subscriber sub1=nh.subscribe("/rfans_curb/road_param",1,get_road_param);

    // Create a ROS publisher for the output point cloud
    // pub = nh.advertise<sensor_msgs::PointCloud2>("/rfans_process/obs", 1);

    // pcl_show::init_pub();
    // Spin
    ros::spin();
}