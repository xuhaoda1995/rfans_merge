#include "rfans_merge.h"

RfansMerge::RfansMerge(cloudPtr cloud_data_16l,cloudPtr cloud_data_32l)
:_cloud_data_16l(cloud_data_16l)
,_cloud_data_32l(cloud_data_32l)
,_cloud_merge(new Cloud)
,trans_cloud_16_(new Cloud)
,trans_cloud_32_(new Cloud)
{

}

void RfansMerge::merge()
{
    // for(auto pt : _cloud_data_16l->points)
    // {
    //     auto pt_trans=global_param::transformPoint(pt,g_LiDAR_16_2_32);
    //     _cloud_merge->push_back(pt_trans);
    // }
    // *_cloud_merge+=*_cloud_data_32l;

	for(auto pt : _cloud_data_16l->points)
    {
        auto pt_trans=global_param::transformPoint(pt,g_LiDAR_pos16);
        pt_trans.intensity=10.0/255;
        _cloud_merge->push_back(pt_trans);
        trans_cloud_16_->push_back(pt_trans);
    }
	for(auto pt : _cloud_data_32l->points)
    {
        auto pt_trans=global_param::transformPoint(pt,g_LiDAR_pos32);
        pt_trans.intensity=20.0/255;
        _cloud_merge->push_back(pt_trans);
        trans_cloud_32_->push_back(pt_trans);
    }
}

Eigen::Matrix4f RfansMerge::getTransformationByICP()
{
	cloudPtr src(new Cloud);
	cloudPtr tgt(new Cloud);
    Cloud output;

	tgt = _cloud_data_32l;
	src = _cloud_data_16l;

	pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.01);
	icp.setMaximumIterations (100);

	icp.setInputSource (src);
	icp.setInputTarget (tgt);
	icp.align(output);

    *_cloud_merge+=output;
    *_cloud_merge+=*_cloud_data_32l;
//	std::cout << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore() << std::endl;
		
	return icp.getFinalTransformation();
}