/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef VANETMOBILITY_HELPER_H
#define VANETMOBILITY_HELPER_H

#include "ns3/vanetmobility.h"
#include "ns3/ptr.h"

namespace ns3
{
namespace vanetmobility
{
/* ... */

class VANETmobilityHelper
{
public:
	VANETmobilityHelper();
	~VANETmobilityHelper();

	Ptr<VANETmobility> GetSumoMObility(std::string,std::string,std::string);
};

} /* namespace vanetmobility */
} /* namespace ns3 */

#endif /* VANETMOBILITY_HELPER_H */

