#include "stdafx.h"

// using namespace global_param;

const int R_FANS_LINE32=32;
const int R_FANS_LINE16=16;

const int OFFSET_IDX32[4]={0,55,14,70};
const int OFFSET_IDX16[2]={0,1};

const double g_LiDAR_pos32[6] = {0, 1.3, 2.99, 2.12, 0.55, 180.0};
const double g_LiDAR_pos16[6] = {0, 1.3, 2.99, 2.12, 0.55, 0.0};
double g_LiDAR_16_2_32[6] = {0, 0, 0, 0, 0, 180.0};

const float laser_vangle32[32]={
   -20.5, -19.5, -18.5, -17.5,
   -16.5, -15.5, -14.5, -13.5,
   -12.5, -11.5, -10.5,  -9.5,
   -8.5,  -7.5,  -6.5,  -5.5,
   -4.5,  -3.5,  -2.5,  -1.5,
   -0.5,   0.5,   1.5,   2.5,
    3.5,   4.5,   5.5,   6.5,
    7.5,   8.5,   9.5,   10.5};

int global_param::getIndexWithOffset(int idx_beam, int idx_sweep, int num_sweep)
{
    int idx = idx_sweep + OFFSET_IDX32[idx_beam % 4];
    if (idx >= num_sweep)
        idx -= num_sweep;
    return getCloudIndex(idx_beam, idx);
}

pcl::PointXYZI global_param::transformPoint(pcl::PointXYZI pt,const double* dof)
{
    double pitchR = dof[3] * M_PI / 180;
    double rollR = dof[4] * M_PI / 180;
    double yawR = dof[5] * M_PI / 180;
    double dx = dof[0];
    double dy = dof[1];
    double dz = dof[2];

    //rotate the points from lidar to vehicle
    pt.y = pt.y * cos(pitchR) - pt.z * sin(pitchR);
    pt.z = pt.y * sin(pitchR) + pt.z * cos(pitchR);

    pt.x = pt.x * cos(rollR) + pt.z * sin(rollR);
    pt.z = -pt.x * sin(rollR) + pt.z * cos(rollR);

    pt.x = pt.x * cos(yawR) - pt.y * sin(yawR);
    pt.y = pt.x * sin(yawR) + pt.y * cos(yawR);

    pt.x += dx;
    pt.y += dy;
    pt.z += dz;

    return pt;
}