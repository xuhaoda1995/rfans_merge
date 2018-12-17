#include "stdafx.h"
#include "rfans_merge.h"
#include "segment.h"
#include "cloud_publisher.h"
#include "PlaneExtractionBySeed.h"

#include <pcl/io/pcd_io.h>
#include <sstream>
#include <pcl/common/time.h>
#include <fstream>

#include <dynamic_reconfigure/server.h>
#include <rfans_merge/merge_Config.h>

#include <pcl/console/parse.h>

using namespace global_utility;
using namespace std;
using namespace pcl::console;

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

void savePoints(cloudPtr cloud_16l, cloudPtr cloud_32l, int frame_num)
{
    char *cwd = NULL;
    cwd = getcwd(NULL, 0);

    const char *path16 = "bkth16l";
    int mode_all = S_IRWXU | S_IRWXG | S_IRWXO;
    mkdir(path16, mode_all);
    chdir(path16);
    std::stringstream ss;
    ss << frame_num << "_line";
    std::string str;
    ss >> str;
    if (mkdir(str.c_str(), mode_all) == 0)
    {
        for (int i = 0; i < cloud_16l->size(); i++)
        {
            int idx = i % 16;
            std::stringstream ss1;
            ss1 << str << "/" << str << "_" << idx << ".txt";
            std::string str1;
            ss1 >> str1;
            ofstream file(str1, ios::app);
            file << cloud_16l->points[i].x << " "
                 << cloud_16l->points[i].y << " "
                 << cloud_16l->points[i].z << endl;
            file.close();
        }
    }

    const char *path32 = "bkth32l";

    chdir(cwd);
    mkdir(path32, mode_all);
    chdir(path32);
    if (mkdir(str.c_str(), mode_all) == 0)
    {
        for (int i = 0; i < cloud_32l->size(); i++)
        {
            int idx = i % 32;
            std::stringstream ss1;
            ss1 << str << "/" << str << "_" << idx << ".txt";
            std::string str1;
            ss1 >> str1;
            ofstream file(str1, ios::app);
            file << cloud_32l->points[i].x << " "
                 << cloud_32l->points[i].y << " "
                 << cloud_32l->points[i].z << endl;
            file.close();
        }
    }
    chdir(cwd);
    free(cwd);
}

void callback(rfans_merge::merge_Config &config, uint32_t level)
{
    g_LiDAR_pos16[0] = config.x_16;
    g_LiDAR_pos16[1] = config.y_16;
    g_LiDAR_pos16[2] = config.z_16;
    g_LiDAR_pos16[3] = config.pitch_16;
    g_LiDAR_pos16[4] = config.roll_16;
    g_LiDAR_pos16[5] = config.yaw_16;

    g_LiDAR_pos32[0] = config.x_32;
    g_LiDAR_pos32[1] = config.y_32;
    g_LiDAR_pos32[2] = config.z_32;
    g_LiDAR_pos32[3] = config.pitch_32;
    g_LiDAR_pos32[4] = config.roll_32;
    g_LiDAR_pos32[5] = config.yaw_32;
}

void usage(char **argv)
{
	cout << "usage: " << argv[0]
		 << " [-s] save two point as specific format"
         << endl
         << " [-h | --help] "
		 << endl;
	cout << argv[0] << " -h | --help : shows this help" << endl;
	return;
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "rfans_merge_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //Run dynamic parameter thread
    dynamic_reconfigure::Server<rfans_merge::merge_Config> server;
    dynamic_reconfigure::Server<rfans_merge::merge_Config>::CallbackType fun_cb;
    fun_cb = boost::bind(&callback, _1, _2);
    server.setCallback(fun_cb);

    if (find_switch(argc, argv, "-h") ||
		find_switch(argc, argv, "--help"))
	{
		usage(argv);
		return (0);
	}
    bool savePath=false;
    if(find_switch(argc,argv,"-s"))
    {
        savePath=true;
    }

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub1 = nh.subscribe("/ns1/rfans_driver/rfans_points", 1, cloudCallback16L);
    ros::Subscriber sub2 = nh.subscribe("/ns2/rfans_driver/rfans_points", 1, cloudCallback32L);

    // Create a ROS publisher for the output point cloud
    // ros::Publisher pub = nh1.advertise<sensor_msgs::PointCloud2>("out", 1);
    CloudPublisher::init_pub(nh_private);

    ros::Rate loop_rate(50);
    int frame_num = 0;

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
        // if (cloud_32l)
        {
            time_t gpsTime16 = cloud_16l->header.stamp & 0xffffffffl;
            time_t gpsTime32 = cloud_32l->header.stamp & 0xffffffffl;

            auto diff_time = abs(gpsTime16 - gpsTime32);
            if (diff_time > 1e6)
            {
                double dtime = diff_time * 1.0 / 1e6;
                char msg_str[100];
                sprintf(msg_str,
                        "time is not synchronous, the differance of time between two cloud is %f s.",
                        dtime);
                ROS_WARN(msg_str);
                continue;
            }

            if(savePath)
            {
                frame_num++;
                savePoints(cloud_16l, cloud_32l, frame_num);
                std::cout<<"have already save "<<frame_num<<" frame points"<<endl;
                continue;
            }

            ros::Time t1 = ros::Time::now();
            RfansMerge rfans_merge(cloud_16l, cloud_32l);

            // auto trans=rfans_merge.getTransformationByICP();
            // auto rotation_matrix=trans.block<3,3>(0,0);
            // auto euler_angles = rotation_matrix.eulerAngles ( 2,1,0 );
            // std::cout<<euler_angles<<std::endl;
            
            rfans_merge.merge();

            CloudPublisher::all_publish(rfans_merge.trans_cloud_16_, rfans_merge.trans_cloud_32_);

            // Segmentation segmentation(cloud_32l);
            // segmentation.segmentAllPlanes();

            // CloudPublisher::all_publish(segmentation.getSegmentPoints(),cloud_32l);

            // PlaneExtractionBySeed planeExtraction(cloud_32l,32);
            // planeExtraction.segmentAllPlanes();

            // CloudPublisher::all_publish(planeExtraction.getPlaneCloud(),cloud_32l);

            cout << "total time is " << ros::Time::now() - t1 << endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
