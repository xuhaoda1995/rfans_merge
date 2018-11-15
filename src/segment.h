#ifndef SEGMENT_H_
#define SEGMENT_H_

#include "stdafx.h"


class Segmentation
{
public:
  Segmentation(cloudPtr cloud);
  ~Segmentation();

  cloudPtr getSegmentPoints() const;

  int segmentAllPlanes();

  std::vector<PlaneSegment> planes_;

private:
  cloudPtr cloud_ori_;
  cloudPtr cloud_plane_;

private:
  int segmentPlane(cloudPtr &plane, std::vector<float> &coeffs, cloudPtr &cloud_residue);

  // int segmentPlaneBySeed(cloudPtr &plane,
  //                        std::vector<float> &coeffs,
  //                        cloudPtr &cloud_residue);
};

#endif