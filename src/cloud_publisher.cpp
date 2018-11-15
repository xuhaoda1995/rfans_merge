#include "cloud_publisher.h"
#include <string>

using namespace std;

const string frame_id = "world";

ros::Publisher CloudPublisher::pub_segment;
ros::Publisher CloudPublisher::pub;

void CloudPublisher::init_pub(ros::NodeHandle nh)
{
    pub_segment = nh.advertise<sensor_msgs::PointCloud2>("cloud_seg", 1);
    pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_ori", 1);
}

void CloudPublisher::all_publish(cloudPtr cloud_obs, cloudPtr cloud)
{
    sensor_msgs::PointCloud2 output1,output2;

    pcl::toROSMsg(*cloud_obs, output1);
    output1.header.frame_id = frame_id;
    pub_segment.publish(output1);

    pcl::toROSMsg(*cloud, output2);
    output2.header.frame_id = frame_id;
    pub.publish(output2);
}