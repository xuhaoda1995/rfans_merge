#include "segment.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <stack>

Segmentation::Segmentation(cloudPtr cloud)
    : cloud_ori_(cloud), cloud_plane_(new Cloud)
{
}

Segmentation::~Segmentation()
{
}

int Segmentation::segmentPlane(cloudPtr &plane, std::vector<float> &coeffs, cloudPtr &cloud_residue)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud_residue);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return 0;
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    coeffs.push_back(coefficients->values[0]);
    coeffs.push_back(coefficients->values[1]);
    coeffs.push_back(coefficients->values[2]);
    coeffs.push_back(coefficients->values[3]);

    std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;

    for (size_t i = 0; i < inliers->indices.size(); ++i)
    {
        int index = inliers->indices[i];
        pcl::PointXYZI temp_point = cloud_ori_->points[index];
        temp_point.intensity = (planes_.size() + 1) * 10;
        plane->push_back(temp_point);
        cloud_residue->erase(cloud_residue->begin() + index);
    }

    return 1;
}

// int Segmentation::segmentPlaneBySeed(cloudPtr &plane,
//                                      std::vector<float> &coeffs,
//                                      cloudPtr &cloud_residue)
// {
//     // creates kdtree object
//     pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

//     // sets our randomly created cloud as the input
//     kdtree.setInputCloud(cloud_residue);

//     std::stack<int> st_plane;
//     st_plane.push(0);

//     while (!st_plane.empty())
//     {
//         auto seed = st_plane.top();
//         st_plane.pop();

//         //create a “searchPoint” which is assigned random coordinates
//         pcl::PointXYZI searchPoint=cloud_residue->points[seed];

//         // K nearest neighbor search
//         int K = 5;
//         std::vector<int> pointIdxNKNSearch(K);
//         std::vector<float> pointNKNSquaredDistance(K);

//         if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
//         {
            
//         }
//     }
// }


int Segmentation::segmentAllPlanes()
{
    cloudPtr temp_cloud(new Cloud);
    std::vector<float> temp_coeff;
    cloudPtr cloud(new Cloud);
    *cloud += *cloud_ori_;
    while (segmentPlane(temp_cloud, temp_coeff, cloud))
    {
        //ignore plane the number of which is less than 10000
        if(temp_cloud->size()<10000)
        {
            continue;
        }
        if(cloud->size()<10000)
        {
            break;
        }
        *cloud_plane_ += *temp_cloud;
        PlaneSegment temp_plane;
        temp_plane.plane.swap(temp_cloud);
        temp_plane.coefficients.swap(temp_coeff);
        planes_.push_back(temp_plane);
    }
}

cloudPtr Segmentation::getSegmentPoints() const
{
    return cloud_plane_;
}