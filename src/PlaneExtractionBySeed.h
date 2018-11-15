#ifndef PLANEEXTRACTIONBYSEED_H_
#define PLANEEXTRACTIONBYSEED_H_

#include "stdafx.h"
#include <stack>

class PlaneExtractionBySeed
{
public:
	PlaneExtractionBySeed(const cloudPtr & cloud, int num_beam);
	~PlaneExtractionBySeed();

	int segmentAllPlanes();

private:

	bool extractRoadPtsBySeed(cloudPtr &plane, std::vector<float> &coeffs, cloudPtr &cloud_residue);

	cloudPtr cloud_;

	int num_face_;
	int num_beam_;

	std::stack<std::pair<int, int> > st_road_;
	cloudPtr road_;
	std::vector<std::vector<int> > container_;

	cloudPtr cloud_plane_;
	std::vector<PlaneSegment> planes_;

	void pushStack(int idxFace, int idxBeam);
	void pushStackInit();
	void pushStackRange(int idxFaceLeft, int idxFaceRight, int idxBeam);

	bool judgePlanePt(int idxFace,int idxBeam);
	double getDistance(const pcl::PointXYZI& p1, const pcl::PointXYZI& p2);

	bool isLine(const pcl::PointXYZI& p1, const pcl::PointXYZI& center, const pcl::PointXYZI& p2);

	bool isLineHorizon(int idxFaceLeft,int idxFaceRight,int idxBeam);
};

#endif