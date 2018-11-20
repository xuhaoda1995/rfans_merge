#include "PlaneExtractionBySeed.h"

using namespace std;
using namespace pcl;
using namespace global_utility;

PlaneExtractionBySeed::PlaneExtractionBySeed(const cloudPtr &cloud, int num_beam)
	: cloud_(cloud)
	  //, cloud_plane_(new Cloud)
	, num_beam_(num_beam)
	, num_sweep_(cloud_->size() / num_beam)
	, num_residue_(cloud_->size())
	, container_(num_sweep_
	, vector<int>(num_beam_, 0))
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

	// cloudPtr cloud(new Cloud);
	// *cloud += *cloud_;
	while (true)
	{
		if(!extractRoadPtsBySeed(temp_cloud, temp_coeff))
		{
			continue;
		}
		//ignore plane the number of which is less than 10000
		if (temp_cloud->size() < 10000)
		{
			continue;
		}
		if(num_residue_ < 10000)
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

bool PlaneExtractionBySeed::extractRoadPtsBySeed(cloudPtr &plane, std::vector<float> &coeffs)
{
	pushStackInit(plane);
	
	while (!st_road_.empty())
	{
		pair<int, int> pt_idx = st_road_.top();
		st_road_.pop();

		judgePlanePt(pt_idx.first, pt_idx.second, plane);
	}

	return getCoeffientOfPlane(plane,coeffs);
}

bool PlaneExtractionBySeed::judgePlanePt(int idxSweep, int idxBeam, cloudPtr& cloud)
{
	int idxSweepLeft = (idxSweep - 50 + num_sweep_) % num_sweep_;
	int idxSweepRight = (idxSweep + 50) % num_sweep_;
	int idxSweepFront = min(31, idxBeam + 1);
	int idxSweepBack = max(0, idxBeam - 1);

	PointXYZI seed = cloud_->at(getIndexWithOffset(idxBeam, idxSweep, num_beam_, num_sweep_));
	PointXYZI seed_l = cloud_->at(getIndexWithOffset(idxBeam, idxSweepLeft, num_beam_, num_sweep_));
	PointXYZI seed_r = cloud_->at(getIndexWithOffset(idxBeam, idxSweepRight, num_beam_, num_sweep_));
	PointXYZI seed_f = cloud_->at(getIndexWithOffset(idxSweepFront, idxSweep, num_beam_, num_sweep_));
	PointXYZI seed_b = cloud_->at(getIndexWithOffset(idxSweepBack, idxSweep, num_beam_, num_sweep_));

	if (isLineHorizon(idxSweepLeft, idxSweepRight, idxBeam))
	{
		pushStackRange(idxSweepLeft, idxSweepRight, idxBeam, cloud);
		//pushStack(idxSweep, idxBeam);
		//pushStack(idxSweepLeft, idxBeam);
		//pushStack(idxSweepRight, idxBeam);
	}
	if (isLineVertical(seed_b, seed, seed_f))
	{
		pushStack(idxSweep, idxBeam, cloud);
		pushStack(idxSweep, idxSweepBack, cloud);
		pushStack(idxSweep, idxSweepFront, cloud);
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

double PlaneExtractionBySeed::getAngle(const pcl::PointXYZI &p1, const pcl::PointXYZI &p2, const pcl::PointXYZI &p3)
{
	pcl::PointXYZI v1, v2;
	double eps = 1e-6;
	v1.x = p1.x - p2.x;
	v1.y = p1.y - p2.y;
	v1.z = p1.z - p2.z;
	v2.x = p3.x - p2.x;
	v2.y - p3.y - p2.y;
	v2.z = p3.z - p2.z;
	double norm1 = getDistance(p1, p2);
	double norm2 = getDistance(p2, p3);
	if (abs(norm1) < eps || abs(norm2) < eps)
	{
		return 0.0;
	}
	return acos((v1.x * v2.x + v1.y * v2.y + v1.z * v2.z) / norm1 / norm2);
}

bool PlaneExtractionBySeed::isLineVertical(const PointXYZI &p1, const PointXYZI &center, const PointXYZI &p2)
{
	double d1 = getDistance(p1, center);
	double d2 = getDistance(p2, center);

	double angle = getAngle(p1, center, p2);
	double d = max(d1, d2);
	double angle_threshold = atan(0.1 / d);
	return abs(angle - M_PI) < angle_threshold;
}

bool PlaneExtractionBySeed::isLineHorizon(int idxSweepLeft, int idxSweepRight, int idxBeam)
{
	if (idxSweepLeft > idxSweepRight)
	{
		idxSweepRight += num_sweep_;
	}
	for (int i = idxSweepLeft; i < idxSweepRight; i++)
	{
		auto tmp = cloud_->at(getIndexWithOffset(idxBeam, i, num_beam_, num_sweep_));
		auto tmp1 = cloud_->at(getIndexWithOffset(idxBeam, i + 1, num_beam_, num_sweep_));
		double d = getDistance(tmp, tmp1);
		if (d > 0.1)
		{
			return false;
		}

		auto tmp_left = cloud_->at(getIndexWithOffset(idxBeam, i - 50, num_beam_, num_sweep_));
		auto tmp_right = cloud_->at(getIndexWithOffset(idxBeam, i + 50, num_beam_, num_sweep_));
		if (abs(getAngle(tmp_left, tmp, tmp_right) - M_PI) > 0.1)
		{
			return false;
		}
	}

	return true;
}

void PlaneExtractionBySeed::pushStack(int idxSweep, int idxBeam, cloudPtr& cloud)
{
	if (container_[idxSweep][idxBeam] == 0)
	{
		int idx = getIndexWithOffset(idxBeam, idxSweep, num_beam_, num_sweep_);
		auto seed = cloud_->at(idx);

		cloud->push_back(seed);
		num_residue_--;

		st_road_.push(pair<int, int>(idxSweep, idxBeam));
		container_[idxSweep][idxBeam] = 1;
	}
}

void PlaneExtractionBySeed::pushStackInit(cloudPtr& cloud)
{
	int idx_sweep, idx_beam, index = -1;
	do
	{
		index++;
		idx_sweep = index / num_beam_;
		idx_beam = index % num_beam_;
	} while (container_[idx_sweep][idx_beam] == 1 || idx_beam == 0 || idx_beam == num_beam_ - 1);

	// st_road_.push(std::pair<int, int>(idx_sweep, idx_beam));
	pushStack(idx_sweep,idx_beam,cloud);
}

void PlaneExtractionBySeed::pushStackRange(int idxSweepLeft, int idxSweepRight, int idxBeam, cloudPtr& cloud)
{
	if (idxSweepLeft < idxSweepRight)
	{
		for (int i = idxSweepLeft; i < idxSweepRight; i++)
		{
			pushStack(i, idxBeam, cloud);
		}
	}
	else
	{
		for (int i = idxSweepLeft; i < num_sweep_; i++)
		{
			pushStack(i, idxBeam, cloud);
		}
		for (int i = 0; i < idxSweepRight; i++)
		{
			pushStack(i, idxBeam, cloud);
		}
	}
}
