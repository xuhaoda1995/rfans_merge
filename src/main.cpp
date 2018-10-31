#include "stdafx.h"
#include "rfans_merge.h"

#include <pcl/io/pcd_io.h>
#include <sstream>
#include <pcl/common/time.h>

using namespace global_param;

cloudPtr g_cloud_32l;
cloudPtr g_cloud_16l;
boost::mutex cloud_mutex32;
boost::mutex cloud_mutex16;

void cloudCallback16L(const sensor_msgs::PointCloud2ConstPtr &input)
{
    //transfer msg to PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_data(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud_data);

    // Do data processing here...
    // auto time_now = pcl::getTime();
    // cloud_data->width = uint32_t(R_FANS_LINE16);
    // cloud_data->height = uint32_t(cloud_data->size() / R_FANS_LINE16);
    // if (cloud_data->size() % R_FANS_LINE16 != 0)
    // {
    //     std::stringstream err_str;
    //     err_str << "the line of rfans is not" << R_FANS_LINE16;
    //     std::cout << err_str.str() << std::endl;
    //     return;
    // }
    for (auto iter = cloud_data->points.begin(); iter != cloud_data->points.end(); iter++)
    {
        if (isnan(iter->x) || isnan(iter->y) || isnan(iter->z))
        {
            cloud_data->erase(iter);
            iter--;
        }
    }

    boost::mutex::scoped_lock lock(cloud_mutex16);
    g_cloud_16l = cloud_data;
}

void cloudCallback32L(const sensor_msgs::PointCloud2ConstPtr &input)
{
    //transfer msg to PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_data(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud_data);

    // Do data processing here...
    // auto time_now = pcl::getTime();
    // cloud_data->width = uint32_t(R_FANS_LINE32);
    // cloud_data->height = uint32_t(cloud_data->size() / R_FANS_LINE32);
    // if (cloud_data->size() % R_FANS_LINE32 != 0)
    // {
    //     std::stringstream err_str;
    //     err_str << "the line of rfans is not" << R_FANS_LINE32;
    //     std::cout << err_str.str() << std::endl;
    //     return;
    // }
    for (auto iter = cloud_data->points.begin(); iter != cloud_data->points.end(); iter++)
    {
        if (isnan(iter->x) || isnan(iter->y) || isnan(iter->z))
        {
            cloud_data->erase(iter);
            iter--;
        }
    }

    boost::mutex::scoped_lock lock(cloud_mutex32);
    g_cloud_32l = cloud_data;
}

void getParam(ros::NodeHandle nh)
{
    double x, y, z, pitch_deg, roll_deg, yaw_deg;
    nh.param<double>("x", x, 0.0);
    g_LiDAR_16_2_32[0] = x;

    nh.param<double>("y", y, 0.0);
    g_LiDAR_16_2_32[1] = y;

    nh.param<double>("z", z, 0.0);
    g_LiDAR_16_2_32[2] = z;

    nh.param<double>("pitch", pitch_deg, 0.0);
    g_LiDAR_16_2_32[3] = pitch_deg;

    nh.param<double>("roll", roll_deg, 0.0);
    g_LiDAR_16_2_32[4] = roll_deg;

    nh.param<double>("yaw", yaw_deg, 170.0);
    g_LiDAR_16_2_32[5] = yaw_deg;

    ROS_INFO("dof6: %lf,%lf,%lf,%lf,%lf,%lf ", g_LiDAR_16_2_32[0], g_LiDAR_16_2_32[1],
             g_LiDAR_16_2_32[2], g_LiDAR_16_2_32[3], g_LiDAR_16_2_32[4], g_LiDAR_16_2_32[5]);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "rfans_merge_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh1("~");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub1 = nh.subscribe("/ns1/rfans_driver/rfans_points", 1, cloudCallback16L);
    ros::Subscriber sub2 = nh.subscribe("/ns2/rfans_driver/rfans_points", 1, cloudCallback32L);

    // Create a ROS publisher for the output point cloud
    ros::Publisher pub = nh1.advertise<sensor_msgs::PointCloud2>("out", 1);

    // // Spin
    // ros::spin();

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        cloudPtr cloud_16l;
        cloudPtr cloud_32l;
        // See if we can get a cloud
        if (cloud_mutex16.try_lock())
        {
            cloud_16l.swap(g_cloud_16l);
            cloud_mutex16.unlock();
        }
        if (cloud_mutex32.try_lock())
        {
            cloud_32l.swap(g_cloud_32l);
            cloud_mutex32.unlock();
        }
        if (cloud_16l && cloud_32l)
        {
            getParam(nh1);

            RfansMerge rfans_merge(cloud_16l, cloud_32l);
            int dtime = abs((int)(cloud_16l->header.stamp - cloud_32l->header.stamp));
            // if (dtime)
            // {
            // auto trans=rfans_merge.getTransformationByICP();
            // auto rotation_matrix=trans.block<3,3>(0,0);
            // auto euler_angles = rotation_matrix.eulerAngles ( 2,1,0 );
            // std::cout<<euler_angles<<std::endl;
            // }
            rfans_merge.merge();

            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(*(rfans_merge.getMergeCloud()), out);
            out.header.frame_id = "world";
            pub.publish(out);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}