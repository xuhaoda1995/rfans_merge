#ifndef CLOUD_PUBLISHER_H_
#define CLOUD_PUBLISHER_H_

#include "stdafx.h"

class CloudPublisher
{
    public:

    static void init_pub(ros::NodeHandle nh);

    static void all_publish(cloudPtr cloud_obs, cloudPtr cloud);

    static ros::Publisher pub_segment;
    // static ros::Publisher pub_curb;
    // static ros::Publisher pub_box;
    static ros::Publisher pub;

};

#endif