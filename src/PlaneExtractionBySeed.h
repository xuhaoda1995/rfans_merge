#ifndef PLANEEXTRACTIONBYSEED_H_
#define PLANEEXTRACTIONBYSEED_H_

#include "stdafx.h"
#include <stack>

class PlaneExtractionBySeed
{
  public:
	PlaneExtractionBySeed(const cloudPtr &cloud, int num_beam);
	~PlaneExtractionBySeed();

	int segmentAllPlanes();

	cloudPtr getPlaneCloud() const { return cloud_plane_; };

  private:
	bool extractRoadPtsBySeed(cloudPtr &plane, std::vector<float> &coeffs);

	cloudPtr cloud_;

	int num_sweep_;
	int num_beam_;
	//the number of points which are not plane points
	int num_residue_;

	std::stack<std::pair<int, int>> st_road_;
	//cloudPtr cloud_plane_;
	std::vector<std::vector<int>> container_;

	cloudPtr cloud_plane_;
	std::vector<PlaneSegment> planes_;

	void pushStack(int idxFace, int idxBeam, cloudPtr& cloud);
	void pushStackInit(cloudPtr& cloud);
	void pushStackRange(int idxFaceLeft, int idxFaceRight, int idxBeam, cloudPtr& cloud);

	bool judgePlanePt(int idxFace, int idxBeam, cloudPtr& cloud);
	double getDistance(const pcl::PointXYZI &p1, const pcl::PointXYZI &p2);
	double getAngle(const pcl::PointXYZI &p1, const pcl::PointXYZI &p2, const pcl::PointXYZI &p3);

	bool isLineVertical(const pcl::PointXYZI &p1, const pcl::PointXYZI &center, const pcl::PointXYZI &p2);

	bool isLineHorizon(int idxFaceLeft, int idxFaceRight, int idxBeam);
};

#endif