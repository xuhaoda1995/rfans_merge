#include "stdafx.h"

// using namespace global_utility;

const int R_FANS_LINE32 = 32;
const int R_FANS_LINE16 = 16;

const int OFFSET_IDX32[4] = {0, 55, 14, 70};
const int OFFSET_IDX16[2] = {0, 1};

double g_LiDAR_pos32[6] = {0, 0, 0, 1.1381, -0.9036, -77.0832};
double g_LiDAR_pos16[6] = {0, 0, 0, 21.23, -2.1647, 174.5744};
double g_LiDAR_16_2_32[6] = {0, 0, 0, 0, 0, 180.0};

const float laser_vangle32[32] = {
    -20.5, -19.5, -18.5, -17.5,
    -16.5, -15.5, -14.5, -13.5,
    -12.5, -11.5, -10.5, -9.5,
    -8.5, -7.5, -6.5, -5.5,
    -4.5, -3.5, -2.5, -1.5,
    -0.5, 0.5, 1.5, 2.5,
    3.5, 4.5, 5.5, 6.5,
    7.5, 8.5, 9.5, 10.5};

int global_utility::getIndexWithOffset(int idx_beam, int idx_sweep, int num_beam, int num_sweep)
{
    int idx = 0;
    switch (num_beam)
    {
    case 16:
    {
        idx = idx_sweep + OFFSET_IDX16[idx_beam % 2];
        idx = (idx + num_sweep) % num_sweep;

        return getCloudIndex(idx_beam, idx, num_beam);
    }
    case 32:
    {
        idx = idx_sweep + OFFSET_IDX32[idx_beam % 4];
        idx = (idx + num_sweep) % num_sweep;

        return getCloudIndex(idx_beam, idx, num_beam);
    }
    default:
    {
        return getCloudIndex(idx_beam, idx, num_beam);
    }
    }
}

pcl::PointXYZI global_utility::transformPoint(pcl::PointXYZI pt, const double *dof)
{
    double pitchR = dof[3] * M_PI / 180;
    double rollR = dof[4] * M_PI / 180;
    double yawR = dof[5] * M_PI / 180;
    double dx = dof[0];
    double dy = dof[1];
    double dz = dof[2];

    pcl::PointXYZI out1, out2, out3;

    out1.y = pt.y * cos(pitchR) - pt.z * sin(pitchR);
    out1.z = pt.y * sin(pitchR) + pt.z * cos(pitchR);
    out1.x = pt.x;

    out2.x = out1.x * cos(rollR) + out1.z * sin(rollR);
    out2.z = -out1.x * sin(rollR) + out1.z * cos(rollR);
    out2.y = out1.y;

    out3.x = out2.x * cos(yawR) - out2.y * sin(yawR);
    out3.y = out2.x * sin(yawR) + out2.y * cos(yawR);
    out3.z = out2.z;

    out3.x += dx;
    out3.y += dy;
    out3.z += dz;

    return out3;
}

bool getCoeffientOfPlane(cloudPtr plane, std::vector<float> coeffs)
{
    if(plane->size() == 0)
    {
        return false;
    }

    //decentroid
    pcl::PointXYZI mean;
    for (auto pt : plane->points)
    {
        mean.x += pt.x;
        mean.y += pt.y;
        mean.z += pt.z;
    }

    mean.x /= plane->size();
    mean.y /= plane->size();
    mean.z /= plane->size();

    std::vector<float> vec;

    for (auto pt : plane->points)
    {
        // pt.x -= mean.x;
        // pt.y -= mean.y;
        // pt.z -= mean.z;
        
        vec.push_back(pt.x-mean.x);
        vec.push_back(pt.y-mean.y);
        vec.push_back(pt.z-mean.z);

    }
    ///////////////////////////////////////

    // std::vector<std::vector<float>> vec{{0.68f, 0.597f}, {-0.211f, 0.823f}, {0.566f, -0.605f}};
    int rows=plane->size();
    const int cols{3};
    
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> m(vec.data(), rows, cols);

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(m, Eigen::ComputeFullV | Eigen::ComputeFullU); // ComputeThinU | ComputeThinV
    Eigen::MatrixXf singular_values = svd.singularValues();
    Eigen::MatrixXf left_singular_vectors = svd.matrixU();
    Eigen::MatrixXf V = svd.matrixV();

    coeffs.clear();
    coeffs.push_back(V(0,2));
    coeffs.push_back(V(1,2));
    coeffs.push_back(V(2,2));

    return true;
}