/*
 * SumoMobility.h
 *
 *  Created on: Dec 31, 2014
 *      Author: chen
 */

#ifndef SUMOMOBILITY_H_
#define SUMOMOBILITY_H_

#include "ns3/vanetmobility.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/RouteElement.h"
#include "ns3/mobility-module.h"

#include <boost/functional/hash.hpp>
#include <unordered_map>

namespace ns3
{
namespace vanetmobility
{
namespace sumomobility
{
class Vector2DHash//For hash purpose
{
public:
	std::size_t operator() (const Vector2D &v) const
	{
        std::size_t seed = 0;
        boost::hash_combine(seed, v.x);
        boost::hash_combine(seed, v.y);
		return seed;
	}
};

class Vector2DEqual//For equal compare
{
public:
  bool operator() (const Vector2D& x, const Vector2D& y) const {return x.x==y.x&&x.y==y.y;}
};

class SumoMobility:
		public VANETmobility
{
public:
	typedef typename std::unordered_map<Vector2D,std::pair<std::string,double>,Vector2DHash,Vector2DEqual > CoordinateToLaneType;

	static TypeId GetTypeId ();
	SumoMobility(std::string,std::string,std::string);
	virtual ~SumoMobility();

	virtual double GetStartTime(uint32_t id);
	virtual double GetStopTime (uint32_t id);

	const RoadMap& getRoadmap() const
	{
		return roadmap;
	}

	const VehicleLoader& getVl() const
	{
		return vl;
	}

	const uint32_t GetNodeSize() const
	{
		return vl.getVehicles().size();
	}

	void Install();

	double GetReadTotalTime()
	{
		return readTotalTime;
	}

	const sumomobility::Trace& GetTrace(uint32_t& Vehicle_ID,Vector& pos) const;

	const CoordinateToLaneType& getCoordinateToLane() const
	{
		return m_CoordinateToLane;
	}

private:
	void LoadTraffic();
	void ForceUpdates (std::vector<Ptr<MobilityModel> > mobilityStack);

	void InitializeCoordinateToLane();

	std::string netxmlpath;
	std::string routexmlpath;
	std::string fcdxmlpath;

	///\name traffic information
	//\{
	RoadMap roadmap;
	VehicleLoader vl;
	double readTotalTime;
	//\}

	//convert the coordinate (x,y) to the lane and offset pair
	CoordinateToLaneType m_CoordinateToLane;

};



} /* namespace sumomobility */
} /* namespace vanetmobility */
} /* namespace ns3 */

#endif /* SUMOMOBILITY_H_ */
