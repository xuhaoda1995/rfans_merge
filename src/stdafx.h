#ifndef STDAFX_H_
#define STDAFX_H_

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
// #include <std_msgs/Float32MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZI> cloud;
typedef cloud::Ptr cloudPtr;
typedef cloud::ConstPtr cloudConstPtr;

extern const int R_FANS_LINE32;
extern const int R_FANS_LINE16;
extern const int OFFSET_IDX32[4];
extern const int OFFSET_IDX16[2];

extern const float laser_vangle32[32];

extern const double g_LiDAR_pos32[6];
extern const double g_LiDAR_pos16[6];
extern const double g_LiDAR_16_2_32[6];

namespace global_param
{

struct PointSrc
{
    float radius;
    float angle;

    PointSrc()
    {
        radius = 0.f;
        angle = 0.f;
    }

    PointSrc(float radius, float angle)
    {
        radius = radius;
        angle = angle;
    }
};

inline int get_idx(int idx_beam, int idx_sweep)
{
    return idx_beam + idx_sweep * R_FANS_LINE32;
}

int get_idx_with_offset(int idx_beam, int idx_sweep, int num_sweep);

pcl::PointXYZI transform_pt(pcl::PointXYZI pt,const double* dof);

} // namespace global_param

#endif