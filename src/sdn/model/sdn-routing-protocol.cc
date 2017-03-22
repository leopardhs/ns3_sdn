/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 Haoliang Chen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-
1307  USA
 *
 * Authors: Haoliang Chen <chl41993@gmail.com>
 */


///
/// \brief Implementation of SDN agent on car side 
/// and related classes.
///
/// This is the main file of this software because SDN's behaviour is
/// implemented here.
///

#define NS_LOG_APPEND_CONTEXT                                   \
  if (GetObject<Node> ()) { std::clog << "[node " << GetObject<Node> ()->GetId () << "] "; }


#include "sdn-routing-protocol.h"
#include "ns3/socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/ipv4-route.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/ipv4-header.h"

#include "stdlib.h" //ABS
#include "string.h"//memset
#include <vector>
#include <algorithm>//find
#include <unistd.h>
#include <deque>
/********** Useful macros **********/

///
/// \brief Gets the delay between a given time and the current time.
///
/// If given time is previous to the current one, then this macro returns
/// a number close to 0. This is used for scheduling events at a certain moment.
///
#define DELAY(time) (((time) < (Simulator::Now ())) ? Seconds (0.000001) : \
                     (time - Simulator::Now () + Seconds (0.000001)))






/********** Miscellaneous constants **********/

/// Maximum allowed jitter.
#define SDN_MAXJITTER          (m_helloInterval.GetSeconds () / 4)
/// Random number between [0-SDN_MAXJITTER] used to jitter SDN packet transmission.
#define JITTER (Seconds (m_uniformRandomVariable->GetValue (0, SDN_MAXJITTER)))


#define SDN_MAX_SEQ_NUM        65535


#define SDN_PORT_NUMBER 419
/// Maximum number of messages per packet.
#define SDN_MAX_MSGS    64

#define ROAD_LENGTH 1000
//#define SIGNAL_RANGE 400.0//m_signal_range

#define INFHOP 2147483647

#define max_car_number 512
#define MAX 10000

namespace ns3 {
namespace sdn {

NS_LOG_COMPONENT_DEFINE ("SdnRoutingProtocol");


/********** SDN controller class **********/

NS_OBJECT_ENSURE_REGISTERED (RoutingProtocol);

TypeId
RoutingProtocol::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::sdn::RoutingProtocol")
    .SetParent<Ipv4RoutingProtocol> ()
    .AddConstructor<RoutingProtocol> ();
  return tid;
}


RoutingProtocol::RoutingProtocol ()
  :
    m_packetSequenceNumber (SDN_MAX_SEQ_NUM),
    m_messageSequenceNumber (SDN_MAX_SEQ_NUM),
    m_helloInterval (Seconds(1)),
    m_rmInterval (Seconds (3)),
    m_minAPInterval (Seconds (1)),
    m_ipv4 (0),
    m_helloTimer (Timer::CANCEL_ON_DESTROY),
    m_rmTimer (Timer::CANCEL_ON_DESTROY),
    m_apTimer (Timer::CANCEL_ON_DESTROY),
    m_queuedMessagesTimer (Timer::CANCEL_ON_DESTROY),
    m_SCHinterface (0),
    m_CCHinterface (0),
    m_nodetype (OTHERS),
    m_appointmentResult (NORMAL),
    m_next_forwarder (uint32_t (0)),
    m_linkEstablished (false),
    m_numArea (0),
    m_isPadding (false),
    m_numAreaVaild (false),
    m_road_length (814),//MagicNumber
    m_signal_range (419)
{
  m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();
}

RoutingProtocol::~RoutingProtocol ()
{
	  //std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
	  sleep(2);
}

void
RoutingProtocol::SetIpv4 (Ptr<Ipv4> ipv4)
{
  NS_ASSERT (ipv4 != 0);
  NS_ASSERT (m_ipv4 == 0);
  NS_LOG_DEBUG ("Created sdn::RoutingProtocol");
  m_helloTimer.SetFunction 
    (&RoutingProtocol::HelloTimerExpire, this);
  m_queuedMessagesTimer.SetFunction 
    (&RoutingProtocol::SendQueuedMessages, this);
  m_rmTimer.SetFunction
    (&RoutingProtocol::RmTimerExpire, this);
  m_apTimer.SetFunction
    (&RoutingProtocol::APTimerExpire, this);

  m_packetSequenceNumber = SDN_MAX_SEQ_NUM;
  m_messageSequenceNumber = SDN_MAX_SEQ_NUM;


  m_ipv4 = ipv4;
}

void RoutingProtocol::DoDispose ()//do in the very end of the simulation
{
  //std::cout << "1" << std::endl;
  m_ipv4 = 0;

  for (std::map< Ptr<Socket>, Ipv4InterfaceAddress >::iterator iter = 
       m_socketAddresses.begin ();
       iter != m_socketAddresses.end (); ++iter)
    {
      iter->first->Close ();
    }
  m_socketAddresses.clear ();
  m_table.clear();
  m_SCHaddr2CCHaddr.clear ();
  //std::cout<<"dodispose"<<std::endl;
  Ipv4RoutingProtocol::DoDispose ();
}

void
RoutingProtocol::PrintRoutingTable (Ptr<OutputStreamWrapper> stream) const
{
  //std::cout << "2" << std::endl;
  std::ostream* os = stream->GetStream ();
  *os << "Destination\t\tMask\t\tNextHop\t\tInterface\tDistance\n";

  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator iter = 
       m_table.begin ();
       iter != m_table.end (); ++iter)
    {
      *os << iter->first << "\t\t";
      *os << iter->second.mask << "\t\t";
      *os << iter->second.nextHop << "\t\t";
      if (Names::FindName (m_ipv4->GetNetDevice (iter->second.interface)) != "")
        {
          *os << 
          Names::FindName (m_ipv4->GetNetDevice (iter->second.interface)) << 
          "\t\t";
        }
      else
        {
          *os << iter->second.interface << "\t\t";
        }
      *os << "\n";
    }
}

void 
RoutingProtocol::DoInitialize ()
{
  //std::cout << "3" << std::endl;
  if (m_CCHmainAddress == Ipv4Address ())
    {
      Ipv4Address loopback ("127.0.0.1");
      uint32_t count = 0;//std::cout<<"012345 "<<std::endl;
      uint32_t count1 = 0;
      for (uint32_t i = 0; i < m_ipv4->GetNInterfaces (); ++i)
        {
          // CAR Use first address as ID
          // LC Use secend address as ID
          Ipv4Address addr = m_ipv4->GetAddress (i, 0).GetLocal ();
          if (addr != loopback)
            {
              if (m_nodetype == CAR)
                {
                  if(count1 == 1)
                      {
                        m_CCHmainAddress = addr;
                        //std::cout<<i<<"234567 "<<addr.Get()<<std::endl; 
                        //m_SCHaddr2CCHaddr.insert(std::map<Ipv4Address, Ipv4Address>::value_type(m_SCHmainAddress, m_CCHmainAddress));
                        //m_SCHaddr2CCHaddr[m_SCHmainAddress] = m_CCHmainAddress;
                        //std::cout<<"666 "<<m_SCHmainAddress.Get()<<" "<<m_CCHmainAddress.Get()<<std::endl;
                        break;
                      }
                  else if(count1 == 0)
                        m_SCHmainAddress = addr;
                  ++count1;
                  //std::cout<<i<<"123456 "<<addr.Get()<<std::endl;
                }
              else
                if (m_nodetype == LOCAL_CONTROLLER)
                  {
                    if (count == 1)
                      {
                        m_CCHmainAddress = addr;
                        //std::cout<<i<<"234567 "<<addr.Get()<<std::endl;
                        break;
                      }
                    ++count;
                  }
            }
        }

      NS_ASSERT (m_CCHmainAddress != Ipv4Address ());
    }

  NS_LOG_DEBUG ("Starting SDN on node " << m_CCHmainAddress);

  Ipv4Address loopback ("127.0.0.1");

  bool canRunSdn = false;
  //Install RecvSDN  Only on CCH channel.
  if(m_interfaceExclusions.find (m_CCHinterface) == m_interfaceExclusions.end ())
    {
      // Create a socket to listen only on this interface
      Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (),
                                                 UdpSocketFactory::GetTypeId ());
      // TRUE
      socket->SetAllowBroadcast (true);
      InetSocketAddress
        inetAddr (m_ipv4->GetAddress (m_CCHinterface, 0).GetLocal (), SDN_PORT_NUMBER);
      socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvSDN,  this));
      if (socket->Bind (inetAddr))
        {
          NS_FATAL_ERROR ("Failed to bind() SDN socket");
        }
      socket->BindToNetDevice (m_ipv4->GetNetDevice (m_CCHinterface));
      m_socketAddresses[socket] = m_ipv4->GetAddress (m_CCHinterface, 0);//  only for CCH
                                                                         //because sendpacket via CCH

      canRunSdn = true;
    }

  Init_NumArea();
  if(canRunSdn)
    {
      HelloTimerExpire ();
      RmTimerExpire ();
      //APTimerExpire ();
      NS_LOG_DEBUG ("SDN on node (Car) " << m_CCHmainAddress << " started");
    }
}

void 
RoutingProtocol::SetCCHInterface (uint32_t interface)
{
  //std::cout << "4" << std::endl;
  //std::cout<<"SetCCHInterface "<<interface<<std::endl;
  m_CCHmainAddress = m_ipv4->GetAddress (interface, 0).GetLocal ();
  m_CCHinterface = interface;
  Ipv4InterfaceAddress temp_if_add = m_ipv4->GetAddress (m_CCHinterface, 0);
  AddEntry (temp_if_add.GetLocal (),
            Ipv4Address (temp_if_add.GetMask ().Get ()),
            temp_if_add.GetLocal (),
            m_CCHinterface);
  //std::cout<<"SetCCHInterface "<<m_CCHmainAddress.Get ()%256<<std::endl;
}

void 
RoutingProtocol::SetSCHInterface (uint32_t interface)
{
 // std::cout << "5" << std::endl;
  //std::cout<<"SetSCHInterface "<<interface<<std::endl;
  m_SCHinterface = interface;
  m_SCHmainAddress = m_ipv4->GetAddress (interface, 0).GetLocal ();
  Ipv4InterfaceAddress temp_if_add = m_ipv4->GetAddress (m_SCHinterface, 0);
  AddEntry (temp_if_add.GetLocal (),
            Ipv4Address (temp_if_add.GetMask ().Get ()),
            temp_if_add.GetLocal (),
            m_SCHinterface);
  //std::cout<<"SetSCHInterface "<<m_SCHmainAddress.Get ()%256<<std::endl;
}

void
RoutingProtocol::SetInterfaceExclusions (std::set<uint32_t> exceptions)
{
  //std::cout << "6" << std::endl;
  m_interfaceExclusions = exceptions;
}

//
// \brief Processes an incoming %SDN packet (Car Side).
void
RoutingProtocol::RecvSDN (Ptr<Socket> socket)
{
  //std::cout << "7" << std::endl;
  //if (m_CCHmainAddress.Get () % 256 > 50)
 // std::cout<<"RecvSDN"<<m_CCHmainAddress.Get () % 256<<std::endl;
  Ptr<Packet> receivedPacket;
  Address sourceAddress;
  receivedPacket = socket->RecvFrom (sourceAddress);//CCH address

  InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom (sourceAddress);
  Ipv4Address senderIfaceAddr = inetSourceAddr.GetIpv4 ();
  Ipv4Address receiverIfaceAddr = m_socketAddresses[socket].GetLocal ();
  NS_ASSERT (receiverIfaceAddr != Ipv4Address ());
  NS_LOG_DEBUG ("SDN node " << m_CCHmainAddress
                << " received a SDN packet from "
                << senderIfaceAddr << " to " << receiverIfaceAddr);

  // All routing messages are sent from and to port RT_PORT,
  // so we check it.
  NS_ASSERT (inetSourceAddr.GetPort () == SDN_PORT_NUMBER);

  Ptr<Packet> packet = receivedPacket;

  sdn::PacketHeader sdnPacketHeader;
  packet->RemoveHeader (sdnPacketHeader);
  NS_ASSERT (sdnPacketHeader.GetPacketLength () >= sdnPacketHeader.GetSerializedSize ());
  uint32_t sizeLeft = sdnPacketHeader.GetPacketLength () - sdnPacketHeader.GetSerializedSize ();

  MessageList messages;

  while (sizeLeft)
    {
      MessageHeader messageHeader;
      if (packet->RemoveHeader (messageHeader) == 0)
        NS_ASSERT (false);

      sizeLeft -= messageHeader.GetSerializedSize ();

      NS_LOG_DEBUG ("SDN Msg received with type "
                    << std::dec << int (messageHeader.GetMessageType ())
                    << " TTL=" << int (messageHeader.GetTimeToLive ())
                    << " SeqNum=" << messageHeader.GetMessageSequenceNumber ());
      messages.push_back (messageHeader);
    }

  m_rxPacketTrace (sdnPacketHeader, messages);
  
  for (MessageList::const_iterator messageIter = messages.begin ();
       messageIter != messages.end (); ++messageIter)
    {
      
      const MessageHeader &messageHeader = *messageIter;
      // If ttl is less than or equal to zero, or
      // the receiver is the same as the originator,
      // the message must be silently dropped
      //if ((messageHeader.GetTimeToLive () == 0)||(IsMyOwnAddress (sdnPacketHeader.originator)))
      if ((messageHeader.GetTimeToLive () == 0)||(messageHeader.GetOriginatorAddress () == m_CCHmainAddress))
        {
          // ignore it
          packet->RemoveAtStart (messageHeader.GetSerializedSize () - messageHeader.GetSerializedSize () );
          continue;
        }

      //std::cout<<"preprocesshm " << messageHeader.GetOriginatorAddress().Get ()<<std::endl;

      switch (messageHeader.GetMessageType ())
        {
        case sdn::MessageHeader::ROUTING_MESSAGE:
          NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                        << "s SDN node " << m_CCHmainAddress
                        << " received Routing message of size " 
                        << messageHeader.GetSerializedSize ());
          //Controller Node should discare Hello_Message
          if (GetType() == CAR || GetType() == OTHERS)
          {
        	  // if(m_mobility->GetPosition().x<=1000.0 && senderIfaceAddr.Get()%256 == 81)//todo
           //        ProcessRm (messageHeader);
        	  // else if(m_mobility->GetPosition().x>1000.0 && senderIfaceAddr.Get()%256 == 84)
        		  ProcessRm (messageHeader);
          }
          break;

        case sdn::MessageHeader::HELLO_MESSAGE:
          NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                        << "s SDN node " << m_CCHmainAddress
                        << " received Routing message of size "
                        << messageHeader.GetSerializedSize ());
          //Car Node should discare Hello_Message
          if (GetType() == LOCAL_CONTROLLER)
          {
            //std::cout   << m_SCHmainAddress <<" RecvDB IP =" << messageIter->GetHello().ID <<std::endl << std::endl;
            ProcessHM (messageHeader,senderIfaceAddr);
          }
          break;

        case sdn::MessageHeader::APPOINTMENT_MESSAGE:
          NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                        << "s SDN node " << m_CCHmainAddress
                        << " received Appointment message of size "
                        << messageHeader.GetSerializedSize ());
          if (GetType() == CAR)
            ProcessAppointment (messageHeader);
          break;
        case sdn::MessageHeader::CARROUTEREQUEST_MESSAGE:
          NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                        << "s SDN node " << m_CCHmainAddress
                        << " received CRREQ message of size "
                        << messageHeader.GetSerializedSize ());
          if (GetType() == LOCAL_CONTROLLER)
            ProcessCRREQ (messageHeader);
          break;
        case sdn::MessageHeader::CARROUTERESPONCE_MESSAGE:
          NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                        << "s SDN node " << m_CCHmainAddress
                        << " received CRREP message of size "
                        << messageHeader.GetSerializedSize ());
          if (GetType() == GLOBAL_CONTROLLER)
            //std::cout << " ProcessCRREP entry" << std::endl;
            ProcessCRREP (messageHeader);
          break;
        case sdn::MessageHeader::LC_INFO_MESSAGE:
          NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                        << "s SDN node " << m_CCHmainAddress
                        << " received CRREP message of size "
                        << messageHeader.GetSerializedSize ());
          if (GetType() == GLOBAL_CONTROLLER)
          {
            ProcessLCINFO(messageHeader);
          }  
          break;
        case sdn::MessageHeader::CRINFO_MESSAGE:
          NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                        << "s SDN node " << m_CCHmainAddress
                        << " received CRREP message of size "
                        << messageHeader.GetSerializedSize ());
          if (GetType() == LOCAL_CONTROLLER)
          {
           // std::cout << "MessageHeader::CRINFO_MESSAGE" << std::endl;
            ProcessCRINFO(messageHeader);
          }
        default:
          NS_LOG_DEBUG ("SDN message type " <<
                        int (messageHeader.GetMessageType ()) <<
                        " not implemented");
        }

    }
    
}// End of RecvSDN

void
RoutingProtocol::ProcessHM (const sdn::MessageHeader &msg,const Ipv4Address &senderIface)
{
  //std::cout << "8" << std::endl;
  /*std::cout<<m_CCHmainAddress.Get ()%256<<" RoutingProtocol::ProcessHM "
      <<msg.GetHello ().ID.Get ()%256<<" m_lc_info size:"
      <<m_lc_info.size ()<<std::endl;
  */
  RemoveTimeOut();
  Vector3D msg_location = msg.GetHello().GetPosition();
  Vector3D lc = m_mobility->GetPosition();
  if(-14 < msg_location.y && msg_location.y < 3014)
  {
    if((int)lc.y % 1000 == 0)//LC的位置在x轴上
    {
        if(msg_location.x > lc.x + 500 || msg_location.x < lc.x-500 
           || msg_location.y > lc.y+14 || msg_location.y < lc.y-14)
            return;
    }
    if((int)lc.x % 1000 == 0)//LC的位置在y轴上
    {
        if(msg_location.y > lc.y + 500 || msg_location.y < lc.y-500 
           || msg_location.x > lc.x+14 || msg_location.x < lc.x-14)
            return;
    }
    //std::cout << "senderIface = " << senderIface << std::endl;
    Ipv4Address ID = msg.GetHello ().ID;//should be SCH address
    m_SCHaddr2CCHaddr[ID] = msg.GetOriginatorAddress();
    //m_SCHaddr2IfaceAddr[ID] = senderIface;
    //std::cout<<"ProcessHM " << msg.GetOriginatorAddress().Get ()<<std::endl;
    //if(m_CCHmainAddress.Get()%256==244 )
  	 // std::cout<<"244ProcessHM " << msg.GetHello ().GetPosition ().x<<std::endl;
    // if(m_CCHmainAddress.Get()%256==81 && msg.GetHello ().GetPosition ().x>1000.0)//todo
  	 //  return;
    // if(m_CCHmainAddress.Get()%256==84 && msg.GetHello ().GetPosition ().x<=1000.0)
   	//   return;
    std::map<Ipv4Address, CarInfo>::iterator it = m_lc_info.find (ID);

    //if(m_CCHmainAddress.Get()%256==244 )
    	 // std::cout<<"！ " << msg.GetHello ().GetPosition ().x<<std::endl;
    //std::cout << m_SCHmainAddress << " " << ID << " "<< msg_location.x <<" "<< msg_location.y << std::endl;
     if (it != m_lc_info.end ())
     {
          CarInfo temp_car;
          temp_car.Active = true;
          temp_car.LastActive = Simulator::Now ();
          temp_car.Position = msg.GetHello ().GetPosition ();
          temp_car.Velocity = msg.GetHello ().GetVelocity ();
          std::pair<Ipv4Address, CarInfo> temp_pair;
          temp_pair = std::make_pair(ID, temp_car);
          if(((int)lc.y % 1000 == 0 && msg_location.x > it->second.Position.x) || ((int)lc.x % 1000 == 0 && msg_location.y > it->second.Position.y))
          {
              std::list<std::pair<Ipv4Address, CarInfo>>::iterator it1 = m_lc_info_p.begin();
              for(; it1 != m_lc_info_p.end(); it1++)
              {
                  if(it1->first == ID)
                    break;
              }
              if(it1 != m_lc_info_p.end())
              {
                  m_lc_info_p.erase(it1);
              }
            //std::cout <<  m_SCHmainAddress << " m_lc_info_p " << ID << " " <<msg_location.x << " " << msg_location.y <<  std::endl;
              if((int)lc.y % 1000 == 0 && msg_location.x > it->second.Position.x)//x up
              {
                  it1 = m_lc_info_p.begin();
                  for(; it1 != m_lc_info_p.end(); it1++)
                  {
                      if(temp_car.Position.x > it1->second.Position.x)
                        continue;
                      else 
                        break;
                  }
                  m_lc_info_p.insert(it1, temp_pair);
              }
              if((int)lc.x % 1000 == 0 && msg_location.y > it->second.Position.y) // y up
              {
                  it1 = m_lc_info_p.begin();
                  for(; it1 != m_lc_info_p.end(); it1++)
                  {
                      if(temp_car.Position.y > it1->second.Position.y)
                        continue;
                      else 
                        break;
                  }
                  m_lc_info_p.insert(it1, temp_pair);
              }
          }
          else
          {
            std::list<std::pair<Ipv4Address, CarInfo>>::iterator it1 = m_lc_info_n.begin();
            for(; it1 != m_lc_info_n.end(); it1++)
            {
                if(it1->first == ID)
                  break;
            }
            if(it1 != m_lc_info_n.end())
            {
                m_lc_info_n.erase(it1);
            }

            //std::cout <<  m_SCHmainAddress << " m_lc_info_n " << ID << " " <<msg_location.x << " " << msg_location.y <<  std::endl;
            if((int)lc.y % 1000 == 0 && msg_location.x < it->second.Position.x)//x down
            {
                  it1 = m_lc_info_n.begin();
                  for(; it1 != m_lc_info_n.end(); it1++)
                  {
                      if(temp_car.Position.x < it1->second.Position.x)
                        continue;
                      else 
                        break;
                  }
                  m_lc_info_n.insert(it1, temp_pair);
            }
            if((int)lc.x % 1000 == 0 && msg_location.y < it->second.Position.y) // y down
            {
                  it1 = m_lc_info_n.begin();
                  for(; it1 != m_lc_info_n.end(); it1++)
                  {
                      if(temp_car.Position.y < it1->second.Position.y)
                        continue;
                      else 
                        break;
                  }
                  m_lc_info_n.insert(it1, temp_pair);
            }
          }
          it->second.Active = true;
          it->second.LastActive = Simulator::Now ();
         // it->second.Position = msg.GetHello ().GetPosition ();
          it->second.Velocity = msg.GetHello ().GetVelocity ();
          it->second.minhop = 0;
     }
     else
     {
        CarInfo CI_temp;
        CI_temp.Active = true;
        CI_temp.LastActive = Simulator::Now ();
        CI_temp.Position = msg.GetHello ().GetPosition ();
        CI_temp.Velocity = msg.GetHello ().GetVelocity ();
        //m_lc_info.insert ( std::pair<Ipv4Address, CarInfo>(ID,CI_temp));
        m_lc_info[ID] = CI_temp;    
     }

 }

   //std::cout << m_lc_info.size() << std::endl;
  // std::cout << m_SCHmainAddress << " "<< msg.GetHello().ID << std::endl;
  // for(std::map<Ipv4Address, CarInfo>::iterator it = m_lc_info_p.begin(); it != m_lc_info_p.end(); it++)
  // {
  //     std::cout << m_SCHmainAddress <<" m_lc_info_p " << it->first << " " << it->second.Position.x << " " <<it->second.Position.y << std::endl; 
  // }
  // for(std::map<Ipv4Address, CarInfo>::iterator it = m_lc_info_n.begin(); it != m_lc_info_n.end(); it++)
  // {
  //     std::cout << m_SCHmainAddress <<" m_lc_info_n " << it->first << " " << it->second.Position.x << " " <<it->second.Position.y << std::endl; 
  // }

}

// \brief Build routing table according to Rm
void
RoutingProtocol::ProcessRm (const sdn::MessageHeader &msg)
{
  //std::cout << "9" << std::endl;
  NS_LOG_FUNCTION (msg);
  
  const sdn::MessageHeader::Rm &rm = msg.GetRm();
  // Check if this rm is for me
  // Ignore rm that ID does not match.
  //std::cout<<"ProcessRm "<<std::endl;
  if (IsMyOwnAddress (rm.ID))//CCH address 
    {
      Time now = Simulator::Now();
      NS_LOG_DEBUG ("@" << now.GetSeconds() << ":Node " << m_CCHmainAddress
                    << "ProcessRm.");

      NS_ASSERT (rm.GetRoutingMessageSize() >= 0);

      Clear();

      SetCCHInterface(m_CCHinterface);
      SetSCHInterface(m_SCHinterface);

      //m_SCHaddr2CCHaddr.insert(std::map<Ipv4Address, Ipv4Address>::value_type(m_SCHmainAddress, m_CCHmainAddress));
      //m_SCHaddr2CCHaddr[m_SCHmainAddress] = m_CCHmainAddress;
      //std::cout<<"233 "<<m_SCHmainAddress.Get()<<" "<<m_CCHmainAddress.Get()<<std::endl;

      for (std::vector<sdn::MessageHeader::Rm::Routing_Tuple>::const_iterator it = rm.routingTables.begin();
            it != rm.routingTables.end();
            ++it)
      {
        //std::cout<<"9999 "<<rm.ID.Get ()<<" "<<rm.ID.Get ()%256<<" "<<it->destAddress.Get ()<<" "<<it->destAddress.Get ()%256<<" "<<it->nextHop.Get ()<<" "<<it->nextHop.Get ()%256<<std::endl;
        AddEntry(it->destAddress,
                 it->mask,
                 it->nextHop,
                 m_SCHinterface);
        Vector pos = m_mobility->GetPosition ();
        std::cout << "IP=" << m_SCHmainAddress << " get a rm packet, my position = "<< pos.x << " "<< pos.y << std::endl;
        std::cout <<"destAddr= " <<it->destAddress << std::endl;
        std::cout <<"nextHop= " <<it->nextHop << std::endl;     
      }
      std::cout << std::endl;  
      // for (std::vector<sdn::MessageHeader::Rm::Routing_Tuple>::const_iterator it = rm.routingTables.begin();
      //     it != rm.routingTables.end();
      //     ++it)
      // {
      //   //std::cout<<"9999 "<<rm.ID.Get ()<<" "<<rm.ID.Get ()%256<<" "<<it->destAddress.Get ()<<" "<<it->destAddress.Get ()%256<<" "<<it->nextHop.Get ()<<" "<<it->nextHop.Get ()%256<<std::endl;
      //     std::cout <<"destAddr= " <<it->destAddress << std::endl;
      //     std::cout <<"nextHop= " <<it->nextHop << std::endl;
      //     std::cout <<"mask= " <<it->mask << std::endl;       
      // }


    }
}

void
RoutingProtocol::ProcessAppointment (const sdn::MessageHeader &msg)
{
  //std::cout << "10" << std::endl;
  NS_LOG_FUNCTION (msg);
  const sdn::MessageHeader::Appointment &appointment = msg.GetAppointment ();
  if (IsMyOwnAddress (appointment.ID))
    {
      switch (appointment.ATField)
      {
        case NORMAL:
          //std::cout<<" \"NORMAL\""<<std::endl;
          break;
        case FORWARDER:
          m_next_forwarder = appointment.NextForwarder;
          //std::cout<<"CAR"<<m_CCHmainAddress.Get () % 256<<"ProcessAppointment";
          //std::cout<<" \"FORWARDER\""<<std::endl;
          //std::cout<<"NextForwarder:"<<m_next_forwarder.Get () % 256<<std::endl;
          break;
        default:
          std::cout<<" ERROR TYPE"<<std::endl;
      }
      m_appointmentResult = appointment.ATField;
    }
}

void
RoutingProtocol::ProcessCRREQ (const sdn::MessageHeader &msg)//LC
{
  //std::cout << "11" << std::endl;
  //std::cout << "ProcessCRREQ" << std::endl;
  NS_LOG_FUNCTION (msg);
  const sdn::MessageHeader::CRREQ &crreq = msg.GetCRREQ ();
  Ipv4Address dest =  crreq.destAddress;
  Ipv4Address source = crreq.sourceAddress;//the car's ip address
  //std::cout << "dest" << dest << std::endl;
  //std::cout << "source" << source << std::endl;
  // if(m_CCHmainAddress.Get()%256 == 81)// need to expand//todo
	 //  return;
  // if(m_lc_info.find(dest)==m_lc_info.end() || m_lc_info.size()<=1)
	 //  return;
  /*for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
       cit != m_lc_info.end (); ++cit)
    {
	  std::cout<<"info"<<cit->first.Get()%256<<std::endl;
    }*/
  //std::cout<<"ProcessCRREQ"<<transferAddress.Get()%256<<std::endl;
  // if(transferAddress == dest)
	 //  return;
  SendCRREP(source, dest, transferAddress);
}

void
RoutingProtocol::ProcessCRREP(const sdn::MessageHeader &msg)//GC compute route 
{
  NS_LOG_FUNCTION (msg);
  const sdn::MessageHeader::CRREP &crrep = msg.GetCRREP ();
  Ipv4Address dest =  crrep.destAddress;
  Ipv4Address source = crrep.sourceAddress;
  Ipv4Address transfer = crrep.transferAddress;
  std::map<Ipv4Address,CarInfo>::iterator it1 = m_gc_info.find(source);
  std::map<Ipv4Address,CarInfo>::iterator it2 = m_gc_info.find(dest);
  std::vector<int> v;
  std::deque<std::vector<int> > route; 
 // std::cout << "dest " << dest << std::endl;
 // std::cout << "source " << source << std::endl;
  // std::cout << "   ";
  // for(int i = 0; i < 16; i++)
  // {
  //   std::cout << i << "  ";
  // }
  // for(int i = 0; i < 16; i++)
  // {
  //       std::cout << std::endl;
  //       std::cout << i << "  ";
  //       for(int j = 0; j < 16; j++)
  //       {
  //         if(gc_lc_graph[i][j].size() != 0)
  //         {
  //           std::cout << "1" << "  ";
  //         }
  //         else
  //         {
  //           std::cout << "0" << "  ";
  //         }
  //       }
  // }
  // std::cout << std::endl;
  // if(it1 == m_gc_info.end())
  // {
  //   std::cout << "11111111" << std::endl;
  // }
  // if(it2 == m_gc_info.end())
  // {
  //   std::cout << "22222222" << std::endl;
  // }
  if(it1 != m_gc_info.end() && it2 != m_gc_info.end()) // get source cch ip
  {
      uint32_t a = (it1->second.Position.y / 1000)*4 + it1->second.Position.x / 1000;
      uint32_t b = (it2->second.Position.y / 1000)*4 + it2->second.Position.x / 1000;
      std::vector<int> t;
      t.push_back(a);
      route.push_back(t);
      //std::cout << "a=" << a << " b=" << b << std::endl;
      while(route.size() != 0)
      {
        uint32_t size = route.size();
        for(uint32_t i = 0; i < size; i++)
        {

          std::vector<int> temp = route.front();
         // std::cout << std::endl;
          // for(int i = 0; i < temp.size(); i++)
          // {
          //   std::cout << temp[i] << " ";
          // }
          route.pop_front();
          for(uint32_t j = 0; j < gc_lc_graph.size(); j++)
          {
            //&& lineNoUsed(temp.back(), j)
            if(gc_lc_graph[temp.back()][j].size() != 0 && std::find(temp.begin(), temp.end(), j) == temp.end() )
            {
              std::vector<int> temp2 = temp;
              temp2.push_back(j);
              if(j == b)
              {
                v = temp2;
                i = size;
                route.clear();
                std::cout << "mypath ";
                for(int i = 0; i < v.size(); i++)
                {
                  std::cout << v[i] << " ";
                }
                std::cout << std::endl;
                break;
              }
              else
              {
                route.push_back(temp2);
              }
            }
          }
        }
      }
  }
  if(v.size() != 0)
  {
    int a = v[0];
    int b = v[1];
    if(gc_lc_graph[a][b].find(1) != gc_lc_graph[a][b].end())
    {
      sdn::MessageHeader msg;
      Time now = Simulator::Now ();
      msg.SetVTime (m_helloInterval);
      msg.SetTimeToLive (41993);//Just MY Birthday.
      msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
      msg.SetMessageType (sdn::MessageHeader::ROUTING_MESSAGE);
      msg.SetOriginatorAddress(m_CCHmainAddress);
      sdn::MessageHeader::Rm &rm = msg.GetRm ();
      rm.ID = source;
      sdn::MessageHeader::Rm::Routing_Tuple rt;
      rt.destAddress = dest;
      Ipv4Address mask("255.255.0.0");
      rt.mask = mask;
      rt.nextHop = gc_lc_graph[a][b][1][1];
      rm.routingTables.push_back (rt);
      rm.routingMessageSize = rm.routingTables.size ();
      QueueMessage (msg, JITTER);
    }
    
    std::pair<Ipv4Address, Ipv4Address> temp_pair;
    temp_pair = std::make_pair(source, dest);
    candidate[temp_pair] = v;
    for(int i = 0; i < v.size()-1; i++)
    {
        std::map<int, std::vector<int>>::iterator it = alreadyUse.find(v[i]);
        if(it == alreadyUse.end())
        {
            std::vector<int> temp_v;
            temp_v.push_back(v[i+1]);
            alreadyUse[v[i]] = temp_v;
        }
        else
        {
            it->second.push_back(v[i+1]);
        }
    }
    SendPath(v, source, dest);
  }
  //std::cout << std::endl;
  // it = m_gc_info.find(dest);
  // if(it != m_gc_info.end()) // get sink sch ip
  // {
  //   std::cout << "Ipv4Address2 " << it->first << std::endl; 
  // }

}

bool RoutingProtocol::lineNoUsed(int start, int next)
{
    std::map<int, std::vector<int>>::iterator it = alreadyUse.find(start);
    if(it == alreadyUse.end())
    {
      return true;
    }
    else
    {
      for(std::vector<int>::iterator it1 = it->second.begin(); it1 != it->second.end(); it1++)
      {
          if(*it1 == next)
          {
            return false;
          }
      }
    }
    return true;
}

void 
RoutingProtocol::SendPath(std::vector<int>& v, Ipv4Address const &source, Ipv4Address const &dest)
{
  if(v.size() != 0)
  {
    int p, n, a, b;
    Ipv4Address ID, sourceAddress, destAddress;
    // Ipv4Address ID("10.1.1.169");
    // Ipv4Address sourceAddress = source;
    // Ipv4Address destAddress = dest;
    // int b = v[1];
    // int a;
    // if(gc_lc_graph[1][b].find(1) != gc_lc_graph[1][b].end())
    // {
    //   Ipv4Address transferAddress = gc_lc_graph[1][b][1][1];
    // }
    // else 
    //   return;

    //SendCRINFO(ID, sourceAddress, destAddress, transferAddress, p, n);

    for(int i = 0; i < v.size()-2; i++)
    {

      a = v[i];
      b = v[i+1];
      int c = v[i+2];
      if(gc_lc_graph[a][b].find(1) != gc_lc_graph[a][b].end())
      {
          ID = gc_lc_graph[a][b][1][0];
      }
      else
        return;
      sourceAddress = source;
      destAddress = dest;
      if(gc_lc_graph[b][c].find(1) != gc_lc_graph[b][c].end())
      {
          transferAddress = gc_lc_graph[b][c][1][1];
      }
      else
        return;
      std::cout << "transferAddress " << transferAddress << std::endl;
      if(a < b)
      {
        p = 1;
        n = 0;
      }
      else
      {
        p = 0;
        n = 1;
      }
      SendCRINFO(ID, sourceAddress, destAddress, transferAddress, p, n);
    }

    int size = v.size();
    a = v[size-2];
    b = v[size-1];
    ID = gc_lc_graph[a][b][1][0];
    sourceAddress = source;
    destAddress = dest;
    transferAddress = dest;
    SendCRINFO(ID, sourceAddress, destAddress, transferAddress, 1, 0);
  }
}

void
RoutingProtocol::SendCRINFO(Ipv4Address const &ID, Ipv4Address const &sourceAddress,
    Ipv4Address const&destAddress, Ipv4Address const &transferAddress, uint32_t p, uint32_t n)
{
  std::cout << "sendCRINFO to " << ID << std::endl;
  sdn::MessageHeader m;
  Time now = Simulator::Now ();
  m.SetVTime (m_helloInterval);
  m.SetTimeToLive (41993);//Just MY Birthday.
  m.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  m.SetMessageType (sdn::MessageHeader::CRINFO_MESSAGE);
  m.SetOriginatorAddress(m_CCHmainAddress);
  sdn::MessageHeader::CRINFO &crinfo = m.GetCRINFO ();
  crinfo.ID = ID;
  crinfo.sourceAddress = sourceAddress;//cch
 // std::cout << "sourceAddress " << sourceAddress << std::endl;

  crinfo.destAddress = destAddress;//sch

 // std::cout << "destAddress " << destAddress << std::endl;
  crinfo.lastAddress = transferAddress;
  crinfo.m_lc_info_p = p;
  crinfo.m_lc_info_n = n;
  QueueMessage (m, JITTER);
}

void
RoutingProtocol::ProcessLCINFO (const sdn::MessageHeader &msg)
{
  int a = 0, b = 0;
  if(gc_lc_graph.size() == 0)
  {
    for(int i = 0; i < 16; i++)
    {
        std::vector<std::map<int, std::vector<Ipv4Address>>> temp;
        for(int i = 0; i < 16; i++)
        {
          std::map<int, std::vector<Ipv4Address>> temp_map;
          temp.push_back(temp_map);
        }
        gc_lc_graph.push_back(temp);
    }
    CarInfo source1;
    Ipv4Address temp1("10.1.1.169");
    source1.Position.x = 1000;
    source1.Position.y = 0;
    source1.Position.z = 0;
    m_gc_info[temp1] = source1;

    CarInfo source2;
    Ipv4Address temp2("192.168.1.169");
    source2.Position.x = 1000;
    source2.Position.y = 0;
    source2.Position.z = 0;
    m_gc_info[temp2] = source2;

    CarInfo sink1;
    Ipv4Address temp3("10.1.1.170");
    sink1.Position.x = 2000;
    sink1.Position.y = 3000;
    sink1.Position.z = 0;
    m_gc_info[temp3] = sink1;

    CarInfo sink2;
    Ipv4Address temp4("192.168.1.170");
    sink2.Position.x = 2000;
    sink2.Position.y = 3000;
    sink2.Position.z = 0;
    m_gc_info[temp4] = sink2;

    CarInfo source3;
    Ipv4Address temp31("10.1.1.171");
    source3.Position.x = 2000;
    source3.Position.y = 0;
    source3.Position.z = 0;
    m_gc_info[temp31] = source3;

    CarInfo source4;
    Ipv4Address temp41("192.168.1.171");
    source4.Position.x = 2000;
    source4.Position.y = 0;
    source4.Position.z = 0;
    m_gc_info[temp41] = source4;

    CarInfo sink3;
    Ipv4Address temp311("10.1.1.172");
    sink3.Position.x = 3000;
    sink3.Position.y = 2000;
    sink3.Position.z = 0;
    m_gc_info[temp311] = sink3;

    CarInfo sink4;
    Ipv4Address temp411("192.168.1.172");
    sink4.Position.x = 3000;
    sink4.Position.y = 2000;
    sink4.Position.z = 0;
    m_gc_info[temp411] = sink4;

    lcinfo_num = 0;
    // for(std::map<Ipv4Address, CarInfo>::iterator it = m_gc_info.begin(); it != m_gc_info.end(); it++)
    // {
    //   std::cout << "Ipv4Address "  << it->first << std::endl;
    // }
  }
  const sdn::MessageHeader::LCINFO &lcinfo = msg.GetLCINFO ();
  //std::cout <<"lcinfolocation "<< lcinfo.x << " " << lcinfo.y << std::endl;
  if(lcinfo.x % 1000 == 0)//LC在y轴平行线上
  {
      a = ((lcinfo.y-500)/1000)*4 + lcinfo.x/1000;
      b = ((lcinfo.y+500)/1000)*4 + lcinfo.x/1000;
  }
  if(lcinfo.y % 1000 == 0)//LC在x轴平行线上
  {
      a = (lcinfo.y/1000)*4 + (lcinfo.x-500)/1000;
      b = (lcinfo.y/1000)*4 + (lcinfo.x+500)/1000;
  }
  gc_lc_graph[a][b].clear();
  gc_lc_graph[b][a].clear();

  if(lcinfo.p == 1)
  {
    std::vector<Ipv4Address> v;// a-->b
    v.push_back(lcinfo.lcAddress);
    v.push_back(lcinfo.m_lc_info_p_begin);
    v.push_back(lcinfo.m_lc_info_p_end);
    gc_lc_graph[a][b][1] = v;

  }
  // else
  // {
  //   std::vector<Ipv4Address> v;
  //   v.push_back(lcinfo.lcAddress);
  //   gc_lc_graph[a][b][0] = v;
  // }
  if(lcinfo.n == 1) // b--> a
  {
    std::vector<Ipv4Address> v;
    v.push_back(lcinfo.lcAddress);
    v.push_back(lcinfo.m_lc_info_n_begin);
    v.push_back(lcinfo.m_lc_info_n_end);
    gc_lc_graph[b][a][1] = v;
  }
  lcinfo_num++;
  if(lcinfo_num % 25 == 0)
  {
      std::cout << "ProcessLCINFO~~~~~~" << std::endl;
      for(std::map<std::pair<Ipv4Address, Ipv4Address>, std::vector<int>> ::iterator it = candidate.begin(); it != candidate.end(); it++)
      {

          int a = it->second[0];
          int b = it->second[1];
          if(a == 1 && gc_lc_graph[a][b].find(1) != gc_lc_graph[a][b].end())
          {
              sdn::MessageHeader msg;
              Time now = Simulator::Now ();
              msg.SetVTime (m_helloInterval);
              msg.SetTimeToLive (41993);//Just MY Birthday.
              msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
              msg.SetMessageType (sdn::MessageHeader::ROUTING_MESSAGE);
              msg.SetOriginatorAddress(m_CCHmainAddress);
              sdn::MessageHeader::Rm &rm = msg.GetRm ();
              Ipv4Address source("192.168.1.169");
              rm.ID = source;
              sdn::MessageHeader::Rm::Routing_Tuple rt;
              Ipv4Address dest("10.1.1.170");
              rt.destAddress = dest;
              Ipv4Address mask("255.255.0.0");
              rt.mask = mask;
              rt.nextHop = gc_lc_graph[a][b][1][1];
              rm.routingTables.push_back (rt);
              rm.routingMessageSize = rm.routingTables.size ();
              QueueMessage (msg, JITTER);
          }
          if(a == 2 && gc_lc_graph[a][b].find(1) != gc_lc_graph[a][b].end())
          {
              sdn::MessageHeader msg;
              Time now = Simulator::Now ();
              msg.SetVTime (m_helloInterval);
              msg.SetTimeToLive (41993);//Just MY Birthday.
              msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
              msg.SetMessageType (sdn::MessageHeader::ROUTING_MESSAGE);
              msg.SetOriginatorAddress(m_CCHmainAddress);
              sdn::MessageHeader::Rm &rm = msg.GetRm ();
              Ipv4Address source("192.168.1.171");
              rm.ID = source;
              sdn::MessageHeader::Rm::Routing_Tuple rt;
              Ipv4Address dest("10.1.1.172");
              rt.destAddress = dest;
              Ipv4Address mask("255.255.0.0");
              rt.mask = mask;
              rt.nextHop = gc_lc_graph[a][b][1][1];
              rm.routingTables.push_back (rt);
              rm.routingMessageSize = rm.routingTables.size ();
              QueueMessage (msg, JITTER);
          }
          SendPath(it->second, (it->first).first, (it->first).second);
      }
      lcinfo_num = 1;
  }
  // else
  // {
  //   std::vector<Ipv4Address> v;
  //   v.push_back(lcinfo.lcAddress);
  //   gc_lc_graph[b][a][0] = v;
  // }
  NS_LOG_FUNCTION (msg);
}

void
RoutingProtocol::ProcessCRINFO (const sdn::MessageHeader &msg)//LC to car and others
{
  NS_LOG_FUNCTION (msg);

  const sdn::MessageHeader::CRINFO &crinfo = msg.GetCRINFO ();
 
  // if(crinfo.sourceAddress == m_CCHmainAddress)
  // {
  //     std::cout << "source " << m_SCHmainAddress << std::endl;
  //     Ipv4Address mask("255.225.0.0");
  //     AddEntry( crinfo.destAddress,
  //               mask,
  //               crinfo.lastAddress,
  //               m_SCHinterface);
  //     std::cout << "IP=" << m_SCHmainAddress <<"get a rm packet" << std::endl;
  //     std::cout << "destAddress=" << crinfo.destAddress << std::endl;
  //     std::cout << "nextHop=" << crinfo.lastAddress << std::endl; 
  // }

  if(crinfo.ID == m_SCHmainAddress)
  {
     std::cout << "ProcessCRINFO LC IP: " << m_SCHmainAddress <<" "<<m_mobility->GetPosition ().x << " " << m_mobility->GetPosition ().y << std::endl;
    if(crinfo.m_lc_info_p == 1 && m_lc_info_p.begin() != m_lc_info_p.end())
    {

        for(std::list<std::pair<Ipv4Address, CarInfo>>::iterator it = m_lc_info_p.begin(); it != m_lc_info_p.end();it++)
        {
          std::cout << " m_lc_info_p: "<<it->first << " " << it->second.Position.x << " " << it->second.Position.y  << std::endl;;
        }
       //  sdn::MessageHeader msg;
       //  Time now = Simulator::Now ();
       //  msg.SetVTime (m_helloInterval);
       //  msg.SetTimeToLive (41993);//Just MY Birthday.
       //  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
       //  msg.SetMessageType (sdn::MessageHeader::ROUTING_MESSAGE);
       //  msg.SetOriginatorAddress(m_CCHmainAddress);
       //  sdn::MessageHeader::Rm &rm = msg.GetRm ();
       //  sdn::MessageHeader::Rm::Routing_Tuple rt;
       //  rm.ID = crinfo.sourceAddress;//sink cch
       // // std::cout << "sourceAddress" <<crinfo.sourceAddress << std::endl;
       //  rt.destAddress = crinfo.destAddress;
       //  Ipv4Address temp("255.255.0.0");
       //  rt.mask = temp;
       //  rt.nextHop = m_lc_info_p.begin()->first;
       //  rm.routingTables.push_back (rt);
       //  // std::cout << "destAddress= " << crinfo.destAddress << std::endl;
       //  // std::cout << "nextHop= " << rt.nextHop << std::endl;
       //  // std::cout << "packet end" << std::endl;
       //  QueueMessage (msg, JITTER);
        for(std::list<std::pair<Ipv4Address, CarInfo>>::iterator it = m_lc_info_p.begin(); it != m_lc_info_p.end();)
        {
            sdn::MessageHeader msg;
            Time now = Simulator::Now ();
            msg.SetVTime (m_helloInterval);
            msg.SetTimeToLive (41993);//Just MY Birthday.
            msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
            msg.SetMessageType (sdn::MessageHeader::ROUTING_MESSAGE);
            msg.SetOriginatorAddress(m_CCHmainAddress);
            sdn::MessageHeader::Rm &rm = msg.GetRm ();
            sdn::MessageHeader::Rm::Routing_Tuple rt;

            rm.ID = m_SCHaddr2CCHaddr[it->first];
            //std::cout << m_SCHmainAddress << " send a rm packet to " << rm.ID << " ip=" << it->first << std::endl;
            rt.destAddress = crinfo.destAddress;
            Ipv4Address temp("255.255.0.0");
            rt.mask = temp;
            it++;
            if(it != m_lc_info_p.end())
            {
                rt.nextHop = it->first;
            }
            else
            {
              rt.nextHop = crinfo.lastAddress;
            }
            rm.routingTables.push_back (rt);
            rm.routingMessageSize = rm.routingTables.size();
            QueueMessage (msg, JITTER); 
        }
    }

    if(crinfo.m_lc_info_n == 1 && m_lc_info_n.begin() != m_lc_info_n.end())
    {
        for(std::list<std::pair<Ipv4Address, CarInfo>>::iterator it = m_lc_info_n.begin(); it != m_lc_info_n.end();it++)
        {
          std::cout << "m_lc_info_n: "<<it->first << " " << it->second.Position.x << " " << it->second.Position.y << std::endl;;
        }
        for(std::list<std::pair<Ipv4Address, CarInfo>>::iterator it = m_lc_info_n.begin(); it != m_lc_info_n.end();)
        {
            sdn::MessageHeader msg;
            Time now = Simulator::Now ();
            msg.SetVTime (m_helloInterval);
            msg.SetTimeToLive (41993);//Just MY Birthday.
            msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
            msg.SetMessageType (sdn::MessageHeader::ROUTING_MESSAGE);
            msg.SetOriginatorAddress(m_CCHmainAddress);
            sdn::MessageHeader::Rm &rm = msg.GetRm ();
            sdn::MessageHeader::Rm::Routing_Tuple rt;

            rm.ID = m_SCHaddr2CCHaddr[it->first];
            //std::cout << m_SCHmainAddress << " send a rm packet to " << rm.ID << " ip=" << it->first << std::endl;
            rt.destAddress = crinfo.destAddress;
            Ipv4Address temp("255.255.0.0");
            rt.mask = temp;
            it++;
            if(it != m_lc_info_n.end())
            {
                rt.nextHop = it->first;
            }
            else
            {
              rt.nextHop = crinfo.lastAddress;
            }
            rm.routingTables.push_back (rt);
            rm.routingMessageSize = rm.routingTables.size();
            QueueMessage (msg, JITTER); 
        }
    }
    
  }
}

void
RoutingProtocol::Clear()
{
  //std::cout << "13" << std::endl;
  NS_LOG_FUNCTION_NOARGS();
  m_table.clear();
  
}

void
RoutingProtocol::AddEntry (const Ipv4Address &dest,
                           const Ipv4Address &mask,
                           const Ipv4Address &next,
                           uint32_t interface)
{
  //std::cout << "14" << std::endl;
  NS_LOG_FUNCTION(this << dest << next << interface << mask << m_CCHmainAddress);
  //std::cout<<"dest:"<<m_next_forwarder.Get () % 256<<std::endl;
  RoutingTableEntry RTE;
  RTE.destAddr = dest;
  RTE.mask = mask;
  RTE.nextHop = next;
  RTE.interface = interface;
  m_table[dest] = RTE;
}

void
RoutingProtocol::AddEntry (const Ipv4Address &dest,
                           const Ipv4Address &mask,
                           const Ipv4Address &next,
                           const Ipv4Address &interfaceAddress)
{
  //std::cout << "15" << std::endl;
  NS_LOG_FUNCTION(this << dest << next << interfaceAddress << mask << m_CCHmainAddress);

  NS_ASSERT (m_ipv4);

  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces(); ++i)
   for (uint32_t j = 0; j< m_ipv4->GetNAddresses(i); ++j)
     {
       if (m_ipv4->GetAddress(i,j).GetLocal() == interfaceAddress)
         {
           AddEntry(dest, mask, next, i);
           return;
         }
     }
  //ERROR NO MATCHING INTERFACES
  NS_ASSERT(false);
}

bool
RoutingProtocol::Lookup(Ipv4Address const &dest,
                        RoutingTableEntry &outEntry) const
{
  //std::cout << "16" << std::endl;
  std::map<Ipv4Address, RoutingTableEntry>::const_iterator it = m_table.find(dest);
      for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator iit = m_table.begin();iit!=m_table.end(); ++iit)
        {
                //if(m_CCHmainAddress.Get()%256 == 244)
               // std::cout<<"1.1 "<<m_SCHmainAddress.Get ()<<"        "<<m_SCHmainAddress.Get ()%256<<"        "<<iit->second.destAddr.Get ()<<"        "<<iit->second.destAddr.Get ()%256<<" 0.0"<<iit->second.nextHop.Get ()<<"        "<<iit->second.nextHop.Get ()%256<<std::endl;
        }
        //std::cout<<std::endl;
    /*if (it == m_table.end())
        std::cout<<"0.0 "<<dest.Get ()<<std::endl;*/
  if (it != m_table.end())
    {
      outEntry = it->second;
      //std::cout<<"！—！"<<it->second.nextHop.Get ()%256<<std::endl;
      return true;
    }
  else
    {
      /*Ipv4Mask MaskTemp;
      uint16_t max_prefix;
      bool max_prefix_meaningful = false;
      for (it = m_table.begin();it!=m_table.end(); ++it)
        {
          MaskTemp.Set (it->second.mask.Get ());//std::cout<<"1.1 "<<it->second.destAddr.Get ()%256<<"        "<<it->second.destAddr.Get ()<<" 0.0"<<it->second.nextHop.Get ()%256<<"0.0"<<it->second.mask.Get ()%256<<std::endl;
          if (MaskTemp.IsMatch (dest, it->second.destAddr))
            {
              if (!max_prefix_meaningful)
                {
                  max_prefix_meaningful = true;
                  max_prefix = MaskTemp.GetPrefixLength ();
                  outEntry = it->second;//std::cout<<"0.01"<<std::endl;
                }
              if (max_prefix_meaningful && (max_prefix < MaskTemp.GetPrefixLength ()))
                {
                  max_prefix = MaskTemp.GetPrefixLength ();
                  outEntry = it->second;
                }
            }
        }
      if (max_prefix_meaningful)
        return true;
      else*/
        return false;
    }

}

void
RoutingProtocol::RemoveEntry (Ipv4Address const &dest)
{
  m_table.erase (dest);
}


bool
RoutingProtocol::RouteInput(Ptr<const Packet> p,
                            const Ipv4Header &header,
                            Ptr<const NetDevice> idev,
                            UnicastForwardCallback ucb,
                            MulticastForwardCallback mcb,
                            LocalDeliverCallback lcb,
                            ErrorCallback ecb)
{
  //std::cout << "17" << std::endl;
  NS_LOG_FUNCTION (this << " " << m_ipv4->GetObject<Node> ()->GetId () << " " << header.GetDestination ());
  // if(header.GetDestination ().Get ()%256 != 255)
  //std::cout<<"RouteInput "<<header.GetSource ().Get ()<< ", "<<m_SCHmainAddress.Get () << ",Dest:"<<header.GetDestination ().Get ()<<std::endl;
  //bool lcb_status = false;
  Ipv4Address dest = header.GetDestination();
  Ipv4Address sour = header.GetSource();

  // Consume self-originated packets
  if (IsMyOwnAddress (sour) == true)
    {
      return true;
    }


  // Local delivery
  NS_ASSERT (m_ipv4->GetInterfaceForDevice (idev) >= 0);
  uint32_t iif = m_ipv4->GetInterfaceForDevice (idev);//SCH dev!
  if (m_ipv4->IsDestinationAddress (dest, iif))
    {
      //Local delivery
      if (!lcb.IsNull ())
        {
          NS_LOG_LOGIC ("Broadcast local delivery to " << dest);
          //std::cout<<"Broadcast local delivery to "<<std::endl;
          lcb (p, header, iif);
          /*if ((m_SCHmainAddress.Get ()%256 == 53)&&(iif=m_SCHinterface))
            {
              std::cout<<m_SCHmainAddress.Get ()%256<<" "<<header.GetDestination ().Get () %256<<std::endl;
              std::cout<<"YES!"<<std::endl;
            }*/
          return true;
        }
      else
        {
          NS_LOG_ERROR ("Unable to deliver packet locally due to null callback");
          ecb (p, header, Socket::ERROR_NOROUTETOHOST);
          return false;
        }
   }
      /*//Broadcast forward
      if ((iif == m_SCHinterface) && (m_nodetype == CAR) && (m_appointmentResult == FORWARDER) && (sour != m_next_forwarder))
        {
          NS_LOG_LOGIC ("Forward broadcast");
          Ptr<Ipv4Route> broadcastRoute = Create<Ipv4Route> ();
          broadcastRoute->SetDestination (dest);
          broadcastRoute->SetGateway (dest);//broadcast
          broadcastRoute->SetOutputDevice (m_ipv4->GetNetDevice (m_SCHinterface));
          broadcastRoute->SetSource (sour);
          //std::cout<<"call ucb"<<std::endl;
          ucb (broadcastRoute, p, header);
        }*/
      
      //Forwardding
      Ptr<Ipv4Route> rtentry;
      RoutingTableEntry entry;
      //std::cout<<"2RouteInput "<<m_SCHmainAddress.Get ()%256 << ",Dest:"<<header.GetDestination ().Get ()<<std::endl;
      //std::cout<<"M_TABLE SIZE "<<m_table.size ()<<std::endl;
      if (Lookup (header.GetDestination (), entry))
      {
          //std::cout<<"found!"<<entry.nextHop.Get()%256<<std::endl;
          uint32_t interfaceIdx = entry.interface;
          rtentry = Create<Ipv4Route> ();
          rtentry->SetDestination (header.GetDestination ());
          // the source address is the interface address that matches
          // the destination address (when multiple are present on the
          // outgoing interface, one is selected via scoping rules)
          NS_ASSERT (m_ipv4);
          uint32_t numOifAddresses = m_ipv4->GetNAddresses (interfaceIdx);
          NS_ASSERT (numOifAddresses > 0);
          Ipv4InterfaceAddress ifAddr;
          if (numOifAddresses == 1) {
              ifAddr = m_ipv4->GetAddress (interfaceIdx, 0);
          } else {
              NS_FATAL_ERROR ("XXX Not implemented yet:  IP aliasing and SDN");
          }
          rtentry->SetSource (ifAddr.GetLocal ());
          rtentry->SetGateway (entry.nextHop);
          rtentry->SetOutputDevice (m_ipv4->GetNetDevice (interfaceIdx));
          NS_LOG_DEBUG ("SDN node " << m_CCHmainAddress
                                 << ": RouteInput for dest=" << header.GetDestination ()
                                 << " --> nextHop=" << entry.nextHop
                                 << " interface=" << entry.interface);
          NS_LOG_DEBUG ("Found route to " << rtentry->GetDestination () << " via nh " << rtentry->GetGateway () << " with source addr " << rtentry->GetSource () << " and output dev " << rtentry->GetOutputDevice ());
          ucb (rtentry, p, header);
      }
      else
      {
          NS_LOG_DEBUG ("SDN node " << m_CCHmainAddress
                                 << ": RouteInput for dest=" << header.GetDestination ()
                                 << " No route to host");
          //std::cout<<"2No route to host"<<std::endl;
      }
        
      return true;

    /*}
  //Drop
  return true;*/
}

void
RoutingProtocol::NotifyInterfaceUp (uint32_t i)
{}
void
RoutingProtocol::NotifyInterfaceDown (uint32_t i)
{}
void
RoutingProtocol::NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address)
{}
void
RoutingProtocol::NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address)
{}

Ptr<Ipv4Route>
RoutingProtocol::RouteOutput (Ptr<Packet> p,
             const Ipv4Header &header,
             Ptr<NetDevice> oif,
             Socket::SocketErrno &sockerr)
{
  //std::cout << "18" << std::endl;
  NS_LOG_FUNCTION (this << " " << m_ipv4->GetObject<Node> ()->GetId () << " " << header.GetDestination () << " " << oif);
  Ptr<Ipv4Route> rtentry;
  RoutingTableEntry entry;
  //std::cout<<"RouteOutput "<<m_SCHmainAddress.Get () << ",Dest:"<<header.GetDestination ().Get ()<<std::endl;
  //std::cout<<"M_TABLE SIZE "<<m_table.size ()<<std::endl;
  if (Lookup (header.GetDestination (), entry))
    {
      //std::cout<<"0found!"<<entry.nextHop.Get()%256<<std::endl;
      uint32_t interfaceIdx = entry.interface;
      if (oif && m_ipv4->GetInterfaceForDevice (oif) != static_cast<int> (interfaceIdx))
        {
          // We do not attempt to perform a constrained routing searchTx_Data_Pkts
          // if the caller specifies the oif; we just enforce that
          // that the found route matches the requested outbound interface
          NS_LOG_DEBUG ("SDN node " << m_SCHmainAddress
                                     << ": RouteOutput for dest=" << header.GetDestination ()
                                     << " Route interface " << interfaceIdx
                                     << " does not match requested output interface "
                                     << m_ipv4->GetInterfaceForDevice (oif));
          sockerr = Socket::ERROR_NOROUTETOHOST;
          std::cout<<"does not match requested output interface"<<std::endl;
          return rtentry;
        }
      rtentry = Create<Ipv4Route> ();
      rtentry->SetDestination (header.GetDestination ());
      // the source address is the interface address that matches
      // the destination address (when multiple are present on the
      // outgoing interface, one is selected via scoping rules)
      NS_ASSERT (m_ipv4);
      uint32_t numOifAddresses = m_ipv4->GetNAddresses (interfaceIdx);
      NS_ASSERT (numOifAddresses > 0);
      Ipv4InterfaceAddress ifAddr;
      if (numOifAddresses == 1) {
          ifAddr = m_ipv4->GetAddress (interfaceIdx, 0);
        } else {
          NS_FATAL_ERROR ("XXX Not implemented yet:  IP aliasing and SDN");
        }
      rtentry->SetSource (ifAddr.GetLocal ());
      rtentry->SetGateway (entry.nextHop);
      rtentry->SetOutputDevice (m_ipv4->GetNetDevice (interfaceIdx));
      sockerr = Socket::ERROR_NOTERROR;
      //std::cout<<"***"<<rtentry->GetDestination ().Get()<<" "<<rtentry->GetGateway ().Get()<<std::endl;
      NS_LOG_DEBUG ("SDN node " << m_SCHmainAddress
                                 << ": RouteOutput for dest=" << header.GetDestination ()
                                 << " --> nextHop=" << entry.nextHop
                                 << " interface=" << entry.interface);
      NS_LOG_DEBUG ("Found route to " << rtentry->GetDestination () << " via nh " << rtentry->GetGateway () << " with source addr " << rtentry->GetSource () << " and output dev " << rtentry->GetOutputDevice ());
    }
  else
    {
      NS_LOG_DEBUG ("SDN node " << m_SCHmainAddress
                                 << ": RouteOutput for dest=" << header.GetDestination ()
                                 << " No route to host");
      sockerr = Socket::ERROR_NOROUTETOHOST;
      SendCRREQ(header.GetDestination());
      //std::cout<<"No route to host"<<std::endl;
    }
  return rtentry;
}

void
RoutingProtocol::Dump ()
{
#ifdef NS3_LOG_ENABLE
  NS_LOG_DEBUG ("Dumpping For" << m_SCHmainAddress);
#endif //NS3_LOG_ENABLE
}

std::vector<RoutingTableEntry>
RoutingProtocol::GetRoutingTableEntries () const
{
  //std::cout << "19" << std::endl;
  std::vector<RoutingTableEntry> rtvt;
  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator it = m_table.begin ();
       it != m_table.end (); ++it)
    {
      rtvt.push_back (it->second);
    }
  return rtvt;
}

int64_t
RoutingProtocol::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uniformRandomVariable->SetStream (stream);
  return 1;
}

uint16_t
RoutingProtocol::GetPacketSequenceNumber ()
{
  m_packetSequenceNumber = (m_packetSequenceNumber + 1) % (SDN_MAX_SEQ_NUM + 1);
  return m_packetSequenceNumber;
}


uint16_t
RoutingProtocol::GetMessageSequenceNumber ()
{
  m_messageSequenceNumber = (m_messageSequenceNumber + 1) % (SDN_MAX_SEQ_NUM + 1);
  return m_messageSequenceNumber;
}

void
RoutingProtocol::HelloTimerExpire ()
{
  //std::cout << "20" << std::endl;
  //std::cout<<"HmTimerExpire "<<m_CCHmainAddress.Get ()%256;
  //std::cout<<", Time:"<<Simulator::Now().GetSeconds ()<<std::endl;
  if (GetType() == CAR)
    {
      SendHello ();      
      //m_SCHaddr2CCHaddr[m_SCHmainAddress] = m_CCHmainAddress;
      //std::cout<<"233 "<<m_SCHmainAddress.Get()<<" "<<m_CCHmainAddress.Get()<<std::endl;
      m_helloTimer.Schedule (m_helloInterval);
    }
}

void
RoutingProtocol::RmTimerExpire ()
{
  //Do nothing.
 // std::cout<<"RmTimerExpire "<<m_CCHmainAddress.Get ()%256;
  //std::cout<<", Time:"<<Simulator::Now().GetSeconds ()<<std::endl;
//std::cout << "21" << std::endl;
  if (GetType () == LOCAL_CONTROLLER)
  {
      ClearAllTables();//std::cout<<"1:"<<std::endl;
      ComputeRoute ();//std::cout<<"2:"<<std::endl;
     // SendRoutingMessage ();//std::cout<<"3:"<<std::endl;
      m_rmTimer.Schedule (m_rmInterval);//std::cout<<"4:"<<std::endl;
  }
}

void
RoutingProtocol::APTimerExpire ()
{
  /*if (GetType() == LOCAL_CONTROLLER)
    {
      ComputeRoute ();
    }*/
}


// SDN packets actually send here.
void
RoutingProtocol::SendPacket (Ptr<Packet> packet,
                             const MessageList &containedMessages)
{
 // std::cout << "22" << std::endl;
  NS_LOG_DEBUG ("SDN node " << m_CCHmainAddress << " sending a SDN packet");
  //std::cout<<"SDN node " << m_CCHmainAddress.Get ()<< " sending a SDN packet"<<std::endl;
  // Add a header
  sdn::PacketHeader header;
  header.originator = this->m_CCHmainAddress;
  header.SetPacketLength (header.GetSerializedSize () + packet->GetSize ());
  header.SetPacketSequenceNumber (GetPacketSequenceNumber ());
  packet->AddHeader (header);

  // Trace it
  m_txPacketTrace (header, containedMessages);

  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator i =
  // Send it
         m_socketAddresses.begin (); i != m_socketAddresses.end (); ++i)
    {
      //std::cout<<"towords " << i->second.GetLocal ()<<std::endl;
      Ipv4Address bcast = i->second.GetLocal ().GetSubnetDirectedBroadcast (i->second.GetMask ());
      i->first->SendTo (packet, 0, InetSocketAddress (bcast, SDN_PORT_NUMBER));
    }
}

void
RoutingProtocol::QueueMessage (const sdn::MessageHeader &message, Time delay)
{
  //std::cout << "23" << std::endl;
   m_queuedMessages.push_back (message);
  if (not m_queuedMessagesTimer.IsRunning ())
    {
      m_queuedMessagesTimer.SetDelay (delay);
      m_queuedMessagesTimer.Schedule ();
    }
}


// NS3 is not multithread, so mutex is unnecessary.
// Here, messages will queue up and send once numMessage is equl to SDN_MAX_MSGS.
// This function will NOT add a header to each message
void
RoutingProtocol::SendQueuedMessages ()
{
  //std::cout << "24" << std::endl;
  Ptr<Packet> packet = Create<Packet> ();
  int numMessages = 0;

  NS_LOG_DEBUG ("SDN node " << m_CCHmainAddress << ": SendQueuedMessages");
  //std::cout<<"SendQueuedMessages  "<<m_CCHmainAddress.Get ()%256 <<std::endl;
  MessageList msglist;

  for (std::vector<sdn::MessageHeader>::const_iterator message = m_queuedMessages.begin ();
       message != m_queuedMessages.end ();
       ++message)
    {
      Ptr<Packet> p = Create<Packet> ();
      p->AddHeader (*message);
      packet->AddAtEnd (p);
      msglist.push_back (*message);
      if (++numMessages == SDN_MAX_MSGS)
        {
          SendPacket (packet, msglist);
          msglist.clear ();
          // Reset variables for next packet
          numMessages = 0;
          packet = Create<Packet> ();
        }
    }

  if (packet->GetSize ())
    {
      SendPacket (packet, msglist);
    }

  m_queuedMessages.clear ();
}

bool
RoutingProtocol::IsMyOwnAddress (const Ipv4Address & a) const
{
  //std::cout << "25" << std::endl;
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
         m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
    {
      Ipv4InterfaceAddress iface = j->second;
      if (a == iface.GetLocal ())
        {
          return true;
        }
    }
  return false;
}

void
RoutingProtocol::SendHello ()
{
  //std::cout << "26" << std::endl;
  NS_LOG_FUNCTION (this);
  sdn::MessageHeader msg;
  Time now = Simulator::Now ();
  msg.SetVTime (m_helloInterval);
  msg.SetTimeToLive (41993);//Just MY Birthday.
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  msg.SetMessageType (sdn::MessageHeader::HELLO_MESSAGE);
  msg.SetOriginatorAddress(m_CCHmainAddress);
 //std::cout<<"SendHello " <<m_mobility->GetPosition ().x<<std::endl;
  sdn::MessageHeader::Hello &hello = msg.GetHello ();
  hello.ID = m_SCHmainAddress;
  Vector pos = m_mobility->GetPosition ();
  Vector vel = m_mobility->GetVelocity ();
  //std::cout << " time:" << now.GetMilliSeconds() << ":" << now.GetMicroSeconds() << " location:" << pos.x << " " << pos.y << " Send a HelloPacket Ip = " << m_SCHmainAddress << std::endl << std::endl;
  hello.SetPosition (pos.x, pos.y, pos.z);
  hello.SetVelocity (vel.x, vel.y, vel.z);

  NS_LOG_DEBUG ( "SDN HELLO_MESSAGE sent by node: " << hello.ID
                 << "   at " << now.GetSeconds() << "s");
  QueueMessage (msg, JITTER);
}

void
RoutingProtocol::SendRoutingMessage ()
{
  //std::cout << "27" << std::endl;
  NS_LOG_FUNCTION (this);
  //std::cout<<"SendRoutingMessage"<<m_CCHmainAddress.Get()%256<<std::endl;
  for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
       cit != m_lc_info.end (); ++cit)
    {
      sdn::MessageHeader msg;
      Time now = Simulator::Now ();
      msg.SetVTime (m_helloInterval);
      msg.SetTimeToLive (41993);//Just MY Birthday.
      msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
      msg.SetMessageType (sdn::MessageHeader::ROUTING_MESSAGE);
      msg.SetOriginatorAddress(m_CCHmainAddress);
      sdn::MessageHeader::Rm &rm = msg.GetRm ();
      //rm.ID = cit->first;//0..0
      //std::cout<<"66666 "<<m_SCHaddr2CCHaddr.size()<<std::endl;
      /*for (std::map<Ipv4Address, Ipv4Address>::const_iterator ttt = m_SCHaddr2CCHaddr.begin ();
           ttt != m_SCHaddr2CCHaddr.end (); ++ttt)
      {
          std::cout<<"6666 "<<ttt->first.Get()<<" "<<ttt->second.Get()<<std::endl;
      }*/
      std::map<Ipv4Address, Ipv4Address>::iterator ttt = m_SCHaddr2CCHaddr.find(cit->first);
      if (ttt != m_SCHaddr2CCHaddr.end ())
      {
          rm.ID = ttt->second;
          //std::cout<<"666666 "<<rm.ID.Get()<<" "<<cit->first.Get()<<std::endl;
      }
      //rm.ID = m_SCHaddr2CCHaddr[cit->first];
      //std::cout<<"666666 "<<rm.ID.Get()<<" "<<cit->first.Get()<<std::endl;
      sdn::MessageHeader::Rm::Routing_Tuple rt;
      for (std::vector<RoutingTableEntry>::const_iterator cit2 = cit->second.R_Table.begin ();
           cit2 != cit->second.R_Table.end (); ++cit2)
        {
          rt.destAddress = cit2->destAddr;
          rt.mask = cit2->mask;
          rt.nextHop = cit2->nextHop;
          //std::cout<<m_CCHmainAddress.Get()%256<<"666666 "<<rm.ID.Get()%256<<" "<<cit2->destAddr.Get()%256<<" "<<cit2->nextHop.Get()%256<<" "<<std::endl;
          rm.routingTables.push_back (rt);
        }
      rm.routingMessageSize = rm.routingTables.size ();
      QueueMessage (msg, JITTER);
    }
}

void
RoutingProtocol::SendAppointment ()
{
  //std::cout << "28" << std::endl;
  NS_LOG_FUNCTION (this);

  for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
       cit != m_lc_info.end (); ++cit)
    {
      sdn::MessageHeader msg;
      Time now = Simulator::Now ();
      msg.SetVTime (m_helloInterval);
      msg.SetTimeToLive (41993);//Just MY Birthday.
      msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
      msg.SetMessageType (sdn::MessageHeader::APPOINTMENT_MESSAGE);
      sdn::MessageHeader::Appointment &appointment = msg.GetAppointment ();
      appointment.ID = cit->first;
      appointment.ATField = cit->second.appointmentResult;
      appointment.NextForwarder = cit->second.ID_of_minhop;
      QueueMessage (msg, JITTER);
    }
}

void
RoutingProtocol::SendCRREQ (Ipv4Address const &destAddress)
{
  //std::cout << "29" << std::endl;
  //Vector pos = m_mobility->GetPosition ();
  NS_LOG_FUNCTION (this);
  //std::cout << "SendCRREQ " << pos.x << " " << pos.y << std::endl;
 // std::cout<<"SendCRREQ "<<std::endl;
  //std::cout << "sourceAddress" << m_CCHmainAddress << std::endl; 
  //std::cout << "destAddress " << destAddress << std::endl;
  sdn::MessageHeader msg;
  Time now = Simulator::Now ();
  msg.SetVTime (m_helloInterval);
  msg.SetTimeToLive (41993);//Just MY Birthday.
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  msg.SetMessageType (sdn::MessageHeader::CARROUTEREQUEST_MESSAGE);
  sdn::MessageHeader::CRREQ &crreq = msg.GetCRREQ ();
  crreq.sourceAddress=m_CCHmainAddress;
  crreq.destAddress=destAddress;
  QueueMessage (msg, JITTER);
}

void
RoutingProtocol::SendCRREP( Ipv4Address const &sourceAddress,
		Ipv4Address const&destAddress, Ipv4Address const &transferAddress)
{
  NS_LOG_FUNCTION (this);
  sdn::MessageHeader msg;
  Time now = Simulator::Now ();
  msg.SetVTime (m_helloInterval);
  msg.SetTimeToLive (41993);//Just MY Birthday.
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  msg.SetMessageType (sdn::MessageHeader::CARROUTERESPONCE_MESSAGE);
  sdn::MessageHeader::CRREP &crrep = msg.GetCRREP ();
  //std::cout << "crrep " << sourceAddress;
  crrep.sourceAddress=sourceAddress;//cch
  crrep.destAddress=destAddress;
  crrep.transferAddress=transferAddress;
  QueueMessage (msg, JITTER);
}


void
RoutingProtocol::SendLCINFO ()
{
  //std::cout << "26" << std::endl;
  NS_LOG_FUNCTION (this);
  sdn::MessageHeader msg;
  Time now = Simulator::Now ();
  msg.SetVTime (m_helloInterval);
  msg.SetTimeToLive (41993);//Just MY Birthday.
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  msg.SetMessageType (sdn::MessageHeader::LC_INFO_MESSAGE);
  msg.SetOriginatorAddress(m_CCHmainAddress);
 // std::cout<<"SendHello " <<m_mobility->GetPosition ().x<<std::endl;
  sdn::MessageHeader::LCINFO &lcinfo = msg.GetLCINFO ();
  //std::cout << "lcAddress" << m_SCHmainAddress << std::endl;
  lcinfo.lcAddress = m_SCHmainAddress;
  if(m_lc_info_p.size() > 2)
  {
     lcinfo.p =1;
     lcinfo.m_lc_info_p_begin = m_lc_info_p.begin()->first;
     lcinfo.m_lc_info_p_end = m_lc_info_p.rbegin()->first;
  }
  else 
  {
    lcinfo.p = 0;
  }
  if(m_lc_info_n.size() > 2)
  {
    lcinfo.n = 1;
    lcinfo.m_lc_info_n_begin = m_lc_info_n.begin()->first;
    lcinfo.m_lc_info_n_end = m_lc_info_n.rbegin()->first;
  }
  else
  {
    lcinfo.n = 0;
  }
  Vector pos = m_mobility->GetPosition ();
  lcinfo.x = pos.x;
  lcinfo.y = pos.y;
  //std::cout << "!!!! " << lcinfo.x << "!!! " << lcinfo.y << std::endl;
  lcinfo.m_gc_info = m_lc_info;
  NS_LOG_DEBUG ( "SDN SendLcInfoMessage sent by node: " << lcinfo.lcAddress
                 << "   at " << now.GetSeconds() << "s");
  QueueMessage (msg, JITTER);
}

void
RoutingProtocol::SetMobility (Ptr<MobilityModel> mobility)
{
  m_mobility = mobility;
}

void
RoutingProtocol::SetType (NodeType nt)
{
  m_nodetype = nt;
}

NodeType
RoutingProtocol::GetType () const
{
  return m_nodetype;
}

typedef struct Edge
{
    int u, v;    // 起点，重点
    int weight;  // 边的权值
} Edge;

void
RoutingProtocol::ComputeRoute ()
{

  // Vector3D lc = m_mobility->GetPosition();

  // std::vector<Ipv4Address> route;
  // if((int)lc.y % 1000 == 0) 
  // {
  //     std::map<double, Ipv4Address> car;
  //     for(std::map<Ipv4Address, CarInfo>::iterator begin = m_lc_info_p.begin(); begin != m_lc_info_p.end(); begin++)
  //     {  
  //         car[begin->second.Position.x-lc.x + 500] = begin->first;
  //     }
  //     car.clear();
  //     for(std::map<Ipv4Address, CarInfo>::iterator begin = m_lc_info_n.begin(); begin != m_lc_info_n.end(); begin++)
  //     {  
  //         car[begin->second.Position.x-lc.x + 500] = begin->first;
  //     }
  // }
  // if((int)lc.x % 1000 == 0) 
  // {
  //     std::map<double, Ipv4Address> car;
  //     for(std::map<Ipv4Address, CarInfo>::iterator begin = m_lc_info_p.begin(); begin != m_lc_info_p.end(); begin++)
  //     {  
  //         car[begin->second.Position.y-lc.y + 500] = begin->first;
  //     }
  //     car.clear();
  //     for(std::map<Ipv4Address, CarInfo>::iterator begin = m_lc_info_n.begin(); begin != m_lc_info_n.end(); begin++)
  //     {  
  //         car[begin->second.Position.y-lc.y+ 500] = begin->first;
  //     }      
  // }
  SendLCINFO();
  // std::cout << std::endl;
  // for(std::map<Ipv4Address, CarInfo>::iterator begin = m_lc_info_p.begin();begin != m_lc_info_p.end(); begin++)
  // {
  //   std::cout << m_SCHmainAddress << " m_lc_info_p " << begin->first  << " "<<  begin->second.Position.x << " " << begin->second.Position.x  << std::endl;
  // }
  // std::cout << std::endl;
  //   for(std::map<Ipv4Address, CarInfo>::iterator begin = m_lc_info_n.begin();begin != m_lc_info_n.end(); begin++)
  // {
  //   std::cout << m_SCHmainAddress <<" m_lc_info_n " << begin->first <<" " <<  begin->second.Position.x << " " << begin->second.Position.y <<std::endl;
  // }
  // std::cout << std::endl;
}//RoutingProtocol::ComputeRoute

void
RoutingProtocol::Do_Init_Compute ()
{
  std::cout << "32" << std::endl;
  std::cout<<"Partition"<<std::endl;
  Partition ();
  std::cout<<"SetN_Init"<<std::endl;
  SetN_Init ();
  std::cout<<"OtherSet_Init"<<std::endl;
  OtherSet_Init ();
  std::cout<<"SelectNode"<<std::endl;
  SelectNode ();
  std::cout<<"Do_Init_Compute DONE"<<std::endl;
}

void
RoutingProtocol::Do_Update ()
{
    //std::cout << "54" << std::endl;
  std::cout<<"ShiftArea"<<std::endl;
  ShiftArea ();
  std::cout<<"AddNewToZero"<<std::endl;
  AddNewToZero ();
  std::cout<<"CalcSetZero"<<std::endl;
  CalcSetZero ();
  std::cout<<"SelectNewNodeInAreaZero"<<std::endl;
  SelectNewNodeInAreaZero ();
  std::cout<<"Do_Update DONE"<<std::endl;
}

void
RoutingProtocol::Partition ()
{
  //std::cout << "33" << std::endl;
  m_Sections.clear ();
  int numArea = GetNumArea();
  for (int i = 0; i < numArea; ++i)
    {
      m_Sections.push_back (std::set<Ipv4Address> ());
    }
  std::cout<<"CheckPonint1"<<std::endl;
  for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
       cit != m_lc_info.end(); ++cit)
    {
      //std::cout<<"cit->first"<<cit->first.Get ()%256<<std::endl;
      //std::cout<<GetArea (cit->second.Position)<<","<<numArea<<std::endl;
      m_Sections[GetArea (cit->second.Position)].insert (cit->first);
    }
  std::cout<<m_lc_info.size ()<<std::endl;
  for (int i = 0; i < numArea; ++i)
    {
      std::cout<<"Section "<<i<<": ";
      for (std::set<Ipv4Address>::const_iterator cit = m_Sections[i].begin ();
           cit != m_Sections[i].end (); ++cit)
        {
          std::cout<<cit->Get ()%256<<",";
        }
      std::cout<<std::endl;
    }

}

void
RoutingProtocol::SetN_Init ()
{
  //std::cout << "34" << std::endl;
  int numArea = GetNumArea();
  for (std::set<Ipv4Address>::const_iterator cit = m_Sections[numArea-1].begin ();
      cit != m_Sections[numArea-1].end (); ++cit)
    {
      std::cout << "111" << std::endl;
      m_lc_info[(*cit)].minhop = 1;
      m_lc_info[(*cit)].ID_of_minhop = Ipv4Address::GetZero ();
      std::cout << "222" << std::endl;
    }
}

void
RoutingProtocol::OtherSet_Init ()
{
  //std::cout << "35" << std::endl;
  int numArea = GetNumArea();
  m_lc_info.clear ();
  for (int area = numArea - 2; area >= 0; --area)
    {
      m_lc_shorthop.clear();
      SortByDistance (area);
      CalcShortHopOfArea (area, area + 1);
      if ((area == numArea - 3) && isPaddingExist ())
        {
          CalcShortHopOfArea (area, area + 2);
        }
      CalcIntraArea (area);
    }
}

void
RoutingProtocol::SortByDistance (int area)
{
  //std::cout << "36" << std::endl;
  m_list4sort.clear ();
  for (std::set<Ipv4Address>::const_iterator cit = m_Sections[area].begin ();
      cit != m_Sections[area].end (); ++cit)
    {
      bool done = false;
      for (std::list<Ipv4Address>::iterator it = m_list4sort.begin ();
           it != m_list4sort.end (); ++it)
        {
          if (m_lc_info[*it].GetPos ().x < m_lc_info[*cit].GetPos ().x)
            {
              m_list4sort.insert (it, *cit);
              done = true;
              break;
            }
        }
      if (!done)
        {
          m_list4sort.push_back (*cit);
        }
    }
}

void
RoutingProtocol::CalcShortHopOfArea (int fromArea, int toArea)
{
    //std::cout << "53" << std::endl;
  for (std::list<Ipv4Address>::const_iterator cit = m_list4sort.begin ();
       cit != m_list4sort.end (); ++cit)
    {
      for (std::set<Ipv4Address>::const_iterator cit2 = m_Sections[toArea].begin ();
           cit2 != m_Sections[toArea].end (); ++cit2)
        {
          m_lc_shorthop[*cit].push_back (GetShortHop (*cit,*cit2));
        }

      UpdateMinHop (*cit);
    }
}

void
RoutingProtocol::UpdateMinHop (const Ipv4Address &ID)
{
  //std::cout << "37" << std::endl;
  uint32_t theminhop = INFHOP;
  Ipv4Address IDofminhop;
  for (std::list<ShortHop>::const_iterator cit = m_lc_shorthop[ID].begin ();
       cit != m_lc_shorthop[ID].end (); ++cit)
    {
      if (cit->hopnumber < theminhop)
        {
          theminhop = cit->hopnumber;
          if (cit->isTransfer)
            {
              IDofminhop = cit->proxyID;
            }
          else
            {
              IDofminhop = cit->nextID;
            }
        }
    }
  m_lc_info[ID].ID_of_minhop = IDofminhop;
  m_lc_info[ID].minhop = theminhop;
}

void
RoutingProtocol::CalcIntraArea (int area)
{
  CalcShortHopOfArea (area, area);
}

void
RoutingProtocol::SelectNode ()
{
  //4-1
  //std::cout << "38" << std::endl;
  ResetAppointmentResult ();
  uint32_t thezero = 0;
  Ipv4Address The_Car(thezero);
  uint32_t minhop_of_tc = INFHOP;

  //First Area
  for (std::set<Ipv4Address>::const_iterator cit = m_Sections[0].begin ();
      cit != m_Sections[0].end (); ++cit)
    {
      CarInfo& temp_info = m_lc_info[*cit];
      if (temp_info.minhop < minhop_of_tc)
        {
          minhop_of_tc = temp_info.minhop;
          The_Car = *cit;
        }
    }
  m_theFirstCar = The_Car;
  Ipv4Address ZERO = Ipv4Address::GetZero ();
  std::cout<<"Chain ";
  while (The_Car != ZERO)
    {
      std::cout<<The_Car.Get () % 256<<",";
      m_lc_info[The_Car].appointmentResult = FORWARDER;
      The_Car = m_lc_info[The_Car].ID_of_minhop;
    }
  std::cout<<std::endl;
}

void
RoutingProtocol::ResetAppointmentResult ()
{
  //std::cout << "39" << std::endl;
  for (std::map<Ipv4Address, CarInfo>::iterator it = m_lc_info.begin ();
       it != m_lc_info.end (); ++it)
    {
      it->second.appointmentResult = NORMAL;
    }
}

void
RoutingProtocol::ShiftArea ()
{
    //std::cout << "52" << std::endl;
  for (int i = GetNumArea () - 1; i>0; --i)
    {
      m_Sections[i] = m_Sections[i-1];
    }
  m_Sections[0].clear ();
}

void
RoutingProtocol::AddNewToZero ()
{
  //std::cout << "40" << std::endl;
  for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
       cit != m_lc_info.end (); ++cit)
    {
      if (GetArea (cit->second.Position) == 0)
        {
          m_Sections[0].insert(cit->first);
        }
    }
}

void
RoutingProtocol::CalcSetZero ()
{
  m_lc_shorthop.clear();
  if (GetNumArea () > 1)
    CalcShortHopOfArea (0,1);
  if ((GetNumArea () == 3)&&(isPaddingExist ()))
    CalcShortHopOfArea (0,2);
  CalcIntraArea (0);
}

void
RoutingProtocol::SelectNewNodeInAreaZero ()
{
  //std::cout << "41" << std::endl;
  uint32_t thezero = 0;
  Ipv4Address The_Car (thezero);
  uint32_t minhop_of_tc = INFHOP;
  for (std::set<Ipv4Address>::const_iterator cit = m_Sections[0].begin ();
       cit != m_Sections[0].end (); ++cit)
    {
      CarInfo& temp_info = m_lc_info[*cit];
      if (temp_info.minhop < minhop_of_tc)
        {
          minhop_of_tc = temp_info.minhop;
          The_Car = *cit;
        }
      else
        if (temp_info.minhop == minhop_of_tc)
          {
            if (temp_info.ID_of_minhop == m_theFirstCar)
              {
                minhop_of_tc = temp_info.minhop;
                The_Car = *cit;
              }
          }
    }

  if (m_lc_info[The_Car].ID_of_minhop == m_theFirstCar)
    {
      m_theFirstCar = The_Car;
      m_lc_info[The_Car].appointmentResult = FORWARDER;
    }
  else
    {
      ResetAppointmentResult ();
      m_theFirstCar = The_Car;
      while (m_lc_info.find (The_Car) != m_lc_info.end ())
        {
          m_lc_info[The_Car].appointmentResult = FORWARDER;
          The_Car = m_lc_info[The_Car].ID_of_minhop;
        }
    }
}

void
RoutingProtocol::Reschedule ()
{
  //std::cout << "42" << std::endl;
  if (m_theFirstCar == Ipv4Address::GetZero ())
    {
      if (m_apTimer.IsRunning ())
        {
          m_apTimer.Remove ();
        }
      m_apTimer.Schedule (m_minAPInterval);
    }
  else
    {
      double vx = m_lc_info[m_theFirstCar].Velocity.x;
      double px = m_lc_info[m_theFirstCar].GetPos ().x;
      double t2l;
      if (vx == 0)
        {
          t2l = 1;
        }
      else
        {
          t2l= (0.5 * m_signal_range - px) / vx;
        }
      if (m_apTimer.IsRunning ())
        {
          m_apTimer.Remove ();
        }
      m_apTimer.Schedule(Seconds(t2l));
    }
}

ShortHop
RoutingProtocol::GetShortHop(const Ipv4Address& IDa, const Ipv4Address& IDb)
{
  //std::cout << "43" << std::endl;
  double const vxa = m_lc_info[IDa].Velocity.x,
               vxb = m_lc_info[IDb].Velocity.x;
  //Predict
  double const pxa = m_lc_info[IDa].GetPos ().x,
               pxb = m_lc_info[IDb].GetPos ().x;
  // time to b left
  double temp;
  if (vxb > 0)
    {
      temp = (m_road_length - pxb) / vxb;
    }
  else
    {
      //b is fixed.
      temp = (m_road_length - pxa) / vxa;
    }
  double const t2bl = temp;
  if ((pxb - pxa < m_signal_range) && (abs((pxb + vxb*t2bl)-(pxa + vxa*t2bl)) < m_signal_range))
    {
      ShortHop sh;
      sh.nextID = IDb;
      sh.hopnumber = m_lc_info[IDb].minhop + 1;
      sh.isTransfer = false;
      return sh;
    }//if ((pxb -  ...
  else
    {
      ShortHop sh;
      sh.isTransfer = true;
      sh.t = 0; // Time when connection loss
      sh.hopnumber = INFHOP;
      if (pxb - pxa < m_signal_range)
        {
          if (vxb > vxa)
            {
              sh.t = (m_signal_range + pxa - pxb) / (vxb - vxa);
            }
          else
            {
              sh.t = (m_signal_range + pxb - pxa) / (vxa - vxb);
            }
        }
      //Find another car
      for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
           cit != m_lc_info.end (); ++cit)
        {
          double const vxc = cit->second.Velocity.x;
          //pxc when t
          double const tpxc = cit->second.GetPos ().x + vxc * sh.t;
          //pxa and pxb when t
          double const tpxa = pxa + vxa * sh.t,
                       tpxb = pxb + vxb * sh.t;
          //t2bl minus t
          double const t2blmt = t2bl - sh.t;
          if ((tpxa<tpxc)&&(tpxc<tpxb))
            {
              if ((abs((tpxb + vxb*t2blmt)-(tpxc + vxc*t2blmt)) < m_signal_range)&&
                  abs((tpxc + vxc*t2blmt)-(tpxa + vxa*t2blmt)) < m_signal_range)
                {
                  sh.IDa = IDa;
                  sh.IDb = IDb;
                  sh.proxyID = cit->first;
                  sh.hopnumber = m_lc_info[IDb].minhop + 2;
                  return sh;
                }//if ((abs((tpxb ...
            }//if ((tpxa ...
        }//for (std::map<I ...
      return sh;
    }//else
}
void
RoutingProtocol::LCAddEntry (const Ipv4Address& ID,
                           const Ipv4Address &dest,
                           const Ipv4Address &mask,
                           const Ipv4Address &next,
                           uint32_t interface)
{
  //std::cout << "44" << std::endl;
  NS_LOG_FUNCTION(this  << ID << dest << next << interface << mask << m_CCHmainAddress);
  //std::cout<<"dest:"<<m_next_forwarder.Get () % 256<<std::endl;
  CarInfo& Entry = m_lc_info[ID];
  //std::cout<<"Interfaces:"<<std::endl;
  RoutingTableEntry RTE;
  RTE.destAddr = dest;
  RTE.mask = mask;
  RTE.nextHop = next;
  RTE.interface = interface;
  //remove repeat
  for(std::vector<RoutingTableEntry>::iterator it=Entry.R_Table.begin();it!=Entry.R_Table.end();++it)
  {
      if(it->destAddr == dest)
      {
              it =  Entry.R_Table.erase(it);//it point to next element;
              --it;
      }
  }  
  Entry.R_Table.push_back(RTE);
}

void
RoutingProtocol::LCAddEntry (const Ipv4Address& ID,
                           const Ipv4Address &dest,
                           const Ipv4Address &mask,
                           const Ipv4Address &next,
                           const Ipv4Address &interfaceAddress)
{
    //std::cout << "51" << std::endl;
  NS_LOG_FUNCTION(this << ID << dest << next << interfaceAddress << mask << m_CCHmainAddress);

  NS_ASSERT (m_ipv4);
  std::cout<<"GetNInterfaces:"<<m_ipv4->GetNInterfaces()<<std::endl;
  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces(); ++i)
   for (uint32_t j = 0; j< m_ipv4->GetNAddresses(i); ++j)
     {
       if (m_ipv4->GetAddress(i,j).GetLocal() == interfaceAddress)
         {
           std::cout<<"GetNInterfaces:"<<i<<std::endl;
           LCAddEntry(ID, dest, mask, next, i);
           return;
         }
     }
  //ERROR NO MATCHING INTERFACES
  NS_ASSERT(false);
}
void
RoutingProtocol::LCAddEntry(const Ipv4Address& ID,
                            const Ipv4Address& dest,
                            const Ipv4Address& mask,
                            const Ipv4Address& next)
{
  //std::cout << "45" << std::endl;
  CarInfo& Entry = m_lc_info[ID];
  RoutingTableEntry RTE;
  RTE.destAddr = dest;
  RTE.mask = mask;
  RTE.nextHop = next;
  RTE.interface = 0;
  //remove repeat
  for(std::vector<RoutingTableEntry>::iterator it=Entry.R_Table.begin();it!=Entry.R_Table.end();++it)
  {
      if(it->destAddr == dest)
      {
              it =  Entry.R_Table.erase(it);//it point to next element;
              --it;
      }
  }  
  Entry.R_Table.push_back (RTE);
}

void
RoutingProtocol::ClearAllTables ()
{
    //std::cout << "50" << std::endl;
  for (std::map<Ipv4Address, CarInfo>::iterator it = m_lc_info.begin (); it!=m_lc_info.end(); ++it)
    {
      for(std::vector<RoutingTableEntry>::iterator iit = it->second.R_Table.begin (); iit!=it->second.R_Table.end(); ++iit)
    {
        if(iit->destAddr != it->first)
        {
                iit = it->second.R_Table.erase(iit);//iit will point to next element;
                --iit;
        }
    }
      //it->second.R_Table.clear ();
    }
}

int
RoutingProtocol::GetArea (Vector3D position) const
{
    //std::cout << "49" << std::endl;
  double &px = position.x;
  double road_length = m_road_length;
  //0.5r ~ r ~ r ~...~ r ~ r ~ last (if length_of_last<=0.5r, last={0.5r}; else last = {padding_area, 0.5r});
  if (px < 0.5*m_signal_range)
    {
      //std::cout<<"RET1"<<std::endl;
      return 0;
    }
  else
    {
      road_length -= 0.5*m_signal_range;
      int numOfTrivialArea = road_length / m_signal_range;
      double remain = road_length - (numOfTrivialArea * m_signal_range);
      if (!(remain>0))
        numOfTrivialArea--;

      px -= 0.5*m_signal_range;
      if (px < numOfTrivialArea * m_signal_range)
        {
          return (px / m_signal_range) + 1;
        }
      else
        {
          if (road_length - px < 0.5*m_signal_range)
            {
              if (isPaddingExist())
                return numOfTrivialArea + 2;
              else
                return numOfTrivialArea + 1;
            }
          else
            {
              return numOfTrivialArea + 1;
            }
        }

    }

}

int
RoutingProtocol::GetNumArea () const
{
  return m_numArea;
}

void
RoutingProtocol::Init_NumArea ()
{
  //std::cout << "47" << std::endl;
  int ret;
  double road_length = m_road_length;
  if (road_length < 0.5*m_signal_range)
    {
      ret = 1;
    }
  else
    {
      road_length -= 0.5*m_signal_range;
      int numOfTrivialArea = road_length / m_signal_range;
      double last_length = road_length - (m_signal_range * numOfTrivialArea);
      if (last_length < 1e-10)//last_length == 0 Devied the last TrivialArea into 2
        {
          ret = 1 + (numOfTrivialArea - 1) + 1 + 1;//First Area + TrivialArea-1 + Padding + LastArea;
          m_isPadding = true;
        }
      else
        if (last_length > 0.5*m_signal_range)//0.5r<last_length<r
          {
            ret = 1 + numOfTrivialArea + 2;//First Area + TrivialArea + paddingArea +LastArea;
            m_isPadding = true;
          }
        else//0<last_length<0.5r
          {
            ret = 1 + numOfTrivialArea + 1;//First Area + TrivialArea + LastArea;
            m_isPadding = false;
          }
    }
  m_numArea = ret;
  m_numAreaVaild = true;
}

bool
RoutingProtocol::isPaddingExist () const
{
  return m_isPadding;
}

void
RoutingProtocol::RemoveTimeOut()
{
  //std::cout << "46" << std::endl;
  Time now = Simulator::Now ();
  std::map<Ipv4Address, CarInfo>::iterator it = m_lc_info.begin ();
  std::vector<Ipv4Address> pendding;
  while (it != m_lc_info.end ())
    {
      if (now.GetSeconds() - it->second.LastActive.GetSeconds () > 2 * m_helloInterval.GetSeconds())
        {
          pendding.push_back (it->first);

          std::list<std::pair<Ipv4Address, CarInfo>>::iterator temp = m_lc_info_p.begin();
          for(; temp != m_lc_info_p.end(); temp++)
          {
            if(temp->first == it->first)
              break;
          }
          if(temp != m_lc_info_p.end())
          {
            m_lc_info_p.erase(temp);
          }
          temp = m_lc_info_n.begin();
          for(; temp != m_lc_info_n.end(); temp++)
          {
            if(temp->first == it->first)
              break;
          }
          if(temp != m_lc_info_n.end())
          {
            m_lc_info_n.erase(temp);
          }
        }
      ++it;
    }
  for (std::vector<Ipv4Address>::iterator it = pendding.begin ();
      it != pendding.end(); ++it)
    {
      //std::cout << "qq1" << std::endl;
      m_lc_info.erase((*it));
      //std::cout << "qq2" << std::endl;
    }
    //std::cout << "RemoveTimeOut end" << std::endl;
}

void
RoutingProtocol::SetSignalRangeNRoadLength (double signal_range, double road_length)
{
  //std::cout << "48" << std::endl; 
  m_signal_range = signal_range;
  m_road_length = road_length;
}

} // namespace sdn
} // namespace ns3


