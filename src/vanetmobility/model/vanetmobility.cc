/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/vanetmobility.h"

namespace ns3
{
namespace vanetmobility
{
NS_OBJECT_ENSURE_REGISTERED (VANETmobility);
/* ... */
TypeId VANETmobility::GetTypeId()
{
	  static TypeId tid = TypeId ("ns3::vanetmobility::VANETmobility")
	    .SetParent<Object> ()
	  ;
	  return tid;
}

VANETmobility::VANETmobility()
{
}

VANETmobility::~VANETmobility()
{
}


} /* namespace vanetmobility */
} /* namespace ns3 */


