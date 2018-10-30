#include "rfans_merge.h"

RfansMerge::RfansMerge(cloudPtr cloud_data_16l,cloudPtr cloud_data_32l)
:_cloud_data_16l(cloud_data_16l)
,_cloud_data_32l(cloud_data_32l)
,_cloud_merge(new cloud)
{

}

void RfansMerge::merge()
{
    for(auto pt : _cloud_data_16l->points)
    {
        auto pt_trans=global_param::transformPoint(pt,g_LiDAR_16_2_32);
        _cloud_merge->push_back(pt_trans);
    }
    *_cloud_merge+=*_cloud_data_32l;
}