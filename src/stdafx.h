#ifndef STDAFX_H_
#define STDAFX_H_

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
// #include <std_msgs/Float32MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
typedef Cloud::Ptr cloudPtr;
typedef Cloud::ConstPtr cloudConstPtr;

extern const int R_FANS_LINE32;
extern const int R_FANS_LINE16;
extern const int OFFSET_IDX32[4];
extern const int OFFSET_IDX16[2];

extern const float laser_vangle32[32];

extern double g_LiDAR_pos32[6];
extern double g_LiDAR_pos16[6];
extern double g_LiDAR_16_2_32[6];

struct PlaneSegment
{
  PlaneSegment()
      : plane(new Cloud)
  {
  }

  std::vector<float> coefficients;
  cloudPtr plane;
};

namespace global_utility
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

inline int getCloudIndex(int idx_beam, int idx_sweep,int num_beam)
{
    return idx_beam + idx_sweep * num_beam;
}

int getIndexWithOffset(int idx_beam, int idx_sweep, int num_beam, int num_sweep);

pcl::PointXYZI transformPoint(pcl::PointXYZI pt, const double* dof);


} // namespace global_utility

bool getCoeffientOfPlane(cloudPtr plane, std::vector<float> coeffs);


#endif