/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/vanetmobility-helper.h"
#include "ns3/SumoMobility.h"

namespace ns3
{
namespace vanetmobility
{

VANETmobilityHelper::VANETmobilityHelper()
{
}

VANETmobilityHelper::~VANETmobilityHelper()
{
}

Ptr<VANETmobility> VANETmobilityHelper::GetSumoMObility(std::string netxml,std::string routexml,std::string fcdxml)
{
	Ptr<VANETmobility> sumoptr = CreateObject<sumomobility::SumoMobility>(netxml,routexml,fcdxml);
	//Ptr<VANETmobility> sumoptr = CreateObject<SumoMobility>(netxml,routexml,fcdxml);
	return sumoptr;
}

} /* namespace vanetmobility */
} /* namespace ns3 */
