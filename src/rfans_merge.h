#ifndef RFANS_MERGE_H_
#define RFANS_MERGE_H_

#include "stdafx.h"
#include <pcl/registration/icp.h>

class RfansMerge
{
  public:
    RfansMerge(cloudPtr cloud_data_16l,cloudPtr cloud_data_32l);

    void merge();

    inline cloudPtr getMergeCloud() const {return _cloud_merge;}

    Eigen::Matrix4f getTransformationByICP();


    cloudPtr trans_cloud_16_;
    cloudPtr trans_cloud_32_;
    
  private:
    cloudPtr _cloud_data_16l;
    cloudPtr _cloud_data_32l;
    cloudPtr _cloud_merge;

};

#endif