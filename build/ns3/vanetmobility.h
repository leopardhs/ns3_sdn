/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef VANETMOBILITY_H
#define VANETMOBILITY_H

#include "ns3/object.h"
#include "ns3/RouteElement.h"

namespace ns3
{
namespace vanetmobility
{

/*
 * This class is for vanetmobility
 *
 */
class VANETmobility:
		public Object
{
public:
	static TypeId GetTypeId ();

	VANETmobility();
	virtual ~VANETmobility();

	virtual double GetStartTime(uint32_t id)=0;
	virtual double GetStopTime (uint32_t id)=0;
	virtual void Install()=0;
	virtual double GetReadTotalTime()=0;
	virtual const uint32_t GetNodeSize() const=0;
	virtual const sumomobility::Trace& GetTrace(uint32_t&,Vector&) const=0;

};


} /* namespace vanetmobility */
} /* namespace ns3 */

#endif /* VANETMOBILITY_H */

