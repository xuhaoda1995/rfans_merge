#include "PlaneExtractionBySeed.h"

using namespace std;
using namespace pcl;
using namespace global_param;

PlaneExtractionBySeed::PlaneExtractionBySeed(const cloudPtr &cloud, int num_beam)
	: cloud_(cloud), road_(new Cloud), num_beam_(num_beam)
	, num_face_(cloud_->size() / num_beam), container_(num_face_, vector<int>(num_beam_, 0))
	, cloud_plane_(new Cloud)
{
}

PlaneExtractionBySeed::~PlaneExtractionBySeed()
{
}

int PlaneExtractionBySeed::segmentAllPlanes()
{
	cloudPtr temp_cloud(new Cloud);
	std::vector<float> temp_coeff;
	cloudPtr cloud(new Cloud);
	*cloud += *cloud_;
	while (extractRoadPtsBySeed(temp_cloud, temp_coeff, cloud))
	{
		//ignore plane the number of which is less than 10000
		if (temp_cloud->size() < 10000)
		{
			continue;
		}
		if (cloud->size() < 10000)
		{
			break;
		}
		*cloud_plane_ += *temp_cloud;
		PlaneSegment temp_plane;
		temp_plane.plane.swap(temp_cloud);
		temp_plane.coefficients.swap(temp_coeff);
		planes_.push_back(temp_plane);
	}

	// for(int i=0;i<cloud_->size();i++)
	// {
	// 	int idx_beam=i%num_beam_;
	// 	int idx_sweep=i/num_beam_;
	// 	if(container_[idx_sweep][idx_beam]==1)
	// 	{
	// 		cloud_plane_->push_back(cloud_->points[i]);
	// 	}
	// }
}

bool PlaneExtractionBySeed::extractRoadPtsBySeed(cloudPtr &plane, std::vector<float> &coeffs, cloudPtr &cloud_residue)
{
	pushStackInit();

	while (!st_road_.empty())
	{
		pair<int, int> pt_idx = st_road_.top();
		st_road_.pop();

		judgePlanePt(pt_idx.first, pt_idx.second);
	}

	return true;
}

bool PlaneExtractionBySeed::judgePlanePt(int idxSweep, int idxBeam)
{
	int idxSweepLeft = (idxSweep - 50 + num_face_) % num_face_;
	int idxSweepRight = (idxSweep + 50) % num_face_;
	int idxSweepFront = min(31, idxBeam + 1);
	int idxSweepBack = max(0, idxBeam - 1);

	PointXYZI seed = cloud_->at(getIndexWithOffset(idxBeam, idxSweep, num_beam_, num_face_));
	PointXYZI seed_l = cloud_->at(getIndexWithOffset(idxBeam, idxSweepLeft, num_beam_, num_face_));
	PointXYZI seed_r = cloud_->at(getIndexWithOffset(idxBeam, idxSweepRight, num_beam_, num_face_));
	PointXYZI seed_f = cloud_->at(getIndexWithOffset(idxSweepFront, idxSweep, num_beam_, num_face_));
	PointXYZI seed_b = cloud_->at(getIndexWithOffset(idxSweepBack, idxSweep, num_beam_, num_face_));

	if (isLineHorizon(idxSweepLeft, idxSweepRight, idxBeam))
	{
		pushStackRange(idxSweepLeft, idxSweepRight, idxBeam);
		//pushStack(idxSweep, idxBeam);
		//pushStack(idxSweepLeft, idxBeam);
		//pushStack(idxSweepRight, idxBeam);
	}
	if (isLine(seed_b, seed, seed_f))
	{
		pushStack(idxSweep, idxBeam);
		pushStack(idxSweep, idxSweepBack);
		pushStack(idxSweep, idxSweepFront);
	}

	return true;
}

double PlaneExtractionBySeed::getDistance(const PointXYZI &p1, const PointXYZI &p2)
{
	double dx, dy, dz;
	dx = p2.x - p1.x;
	dy = p2.y - p1.y;
	dz = p2.z - p1.z;
	return sqrt(dx * dx + dy * dy + dz * dz);
}

bool PlaneExtractionBySeed::isLine(const PointXYZI &p1, const PointXYZI &center, const PointXYZI &p2)
{
	PointXYZI v1, v2;
	v1.x = p1.x - center.x;
	v1.y = p1.y - center.y;
	v1.z = p1.z - center.z;
	v2.x = p2.x - center.x;
	v2.y = p2.y - center.y;
	v2.z = p2.z - center.z;

	double d1 = getDistance(p1, center);
	double d2 = getDistance(p2, center);
	if (d1 == 0 || d2 == 0)
	{
		return false;
	}

	double angle = acos((v1.x * v2.x + v1.y * v2.y + v1.z * v2.z) / d1 / d2);
	double d = max(d1, d2);
	double angle_threshold = atan(0.1 / d);
	return abs(angle - M_PI) < angle_threshold;
}

bool PlaneExtractionBySeed::isLineHorizon(int idxSweepLeft, int idxSweepRight, int idxBeam)
{
	//vector<PointXYZI> tmp_ptVec;
	float z_min = 100;
	float z_max = -100;

	for (int i = idxSweepLeft; i < idxSweepRight; i++)
	{
		auto tmp = cloud_->at(getIndexWithOffset(idxBeam, i, num_beam_, num_face_));
		if (tmp.z < z_min)
			z_min = tmp.z;
		if (tmp.z > z_max)
			z_max = tmp.z;

		if (i + 1 == idxSweepRight)
			continue;
		auto tmp1 = cloud_->at(getIndexWithOffset(idxBeam, i + 1, num_beam_, num_face_));
		double d = getDistance(tmp, tmp1);
		if (d > 0.1)
		{
			return false;
		}

		float dz = abs(tmp.z - tmp1.z);
	}

	if (z_max - z_min > 0.1)
	{
		return false;
	}

	return true;
}

void PlaneExtractionBySeed::pushStack(int idxSweep, int idxBeam)
{
	if (container_[idxSweep][idxBeam] == 0)
	{
		int idx = getIndexWithOffset(idxBeam, idxSweep, num_beam_, num_face_);
		auto seed = cloud_->at(idx);

		road_->push_back(seed);

		st_road_.push(pair<int, int>(idxSweep, idxBeam));
		container_[idxSweep][idxBeam] = 1;
	}
}

void PlaneExtractionBySeed::pushStackInit()
{
	int idx_sweep, idx_beam, index = -1;
	do
	{
		index++;
		idx_sweep = index / num_beam_;
		idx_beam = index % num_beam_;
	} while (container_[idx_sweep][idx_beam] == 1);

	st_road_.push(std::pair<int,int>(idx_sweep,idx_beam));
}

void PlaneExtractionBySeed::pushStackRange(int idxSweepLeft, int idxSweepRight, int idxBeam)
{
	if (idxSweepLeft < idxSweepRight)
	{
		for (int i = idxSweepLeft; i < idxSweepRight; i++)
		{
			pushStack(i, idxBeam);
		}
	}
	else
	{
		for (int i = idxSweepLeft; i < num_face_; i++)
		{
			pushStack(i, idxBeam);
		}
		for (int i = 0; i < idxSweepRight; i++)
		{
			pushStack(i, idxBeam);
		}
	}
}
