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
/// \brief Implementation of DB agent on car side
/// and related classes.
///
/// This is the main file of this software because DB's behaviour is
/// implemented here.
///

#define NS_LOG_APPEND_CONTEXT                                   \
  if (GetObject<Node> ()) { std::clog << "[node " << GetObject<Node> ()->GetId () << "] "; }


#include "db-routing-protocol.h"
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
#define DB_MAXJITTER          (m_helloInterval.GetSeconds () / 4)
/// Random number between [0-DB_MAXJITTER] used to jitter DB packet transmission.
#define JITTER (Seconds (m_uniformRandomVariable->GetValue (0, DB_MAXJITTER)))


#define DB_MAX_SEQ_NUM        65535


#define DB_PORT_NUMBER 419
/// Maximum number of messages per packet.
#define DB_MAX_MSGS    64

#define ROAD_LENGTH 1000
//#define SIGNAL_RANGE 400.0//m_signal_range

#define INFHOP 2147483647

#define max_car_number 256
#define MAX 10000

namespace ns3 {
namespace db {

NS_LOG_COMPONENT_DEFINE ("DbRoutingProtocol");


/********** DB controller class **********/

NS_OBJECT_ENSURE_REGISTERED (RoutingProtocol);

TypeId
RoutingProtocol::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::db::RoutingProtocol")
    .SetParent<Ipv4RoutingProtocol> ()
    .AddConstructor<RoutingProtocol> ();
  return tid;
}


RoutingProtocol::RoutingProtocol ()
  :
    m_packetSequenceNumber (DB_MAX_SEQ_NUM),
    m_messageSequenceNumber (DB_MAX_SEQ_NUM),
    m_helloInterval (Seconds(1)),
    m_rmInterval (Seconds (10)),
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
}



void
RoutingProtocol::SetIpv4 (Ptr<Ipv4> ipv4)
{
  NS_ASSERT (ipv4 != 0);
  NS_ASSERT (m_ipv4 == 0);
  NS_LOG_DEBUG ("Created db::RoutingProtocol");
  m_helloTimer.SetFunction 
    (&RoutingProtocol::HelloTimerExpire, this);
  m_queuedMessagesTimer.SetFunction 
    (&RoutingProtocol::SendQueuedMessages, this);
  m_rmTimer.SetFunction
    (&RoutingProtocol::RmTimerExpire, this);
  m_apTimer.SetFunction
    (&RoutingProtocol::APTimerExpire, this);

  m_packetSequenceNumber = DB_MAX_SEQ_NUM;
  m_messageSequenceNumber = DB_MAX_SEQ_NUM;


  m_ipv4 = ipv4;
}

void RoutingProtocol::DoDispose ()//do in the very end of the simulation
{
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

  NS_LOG_DEBUG ("Starting DB on node " << m_CCHmainAddress);

  Ipv4Address loopback ("127.0.0.1");

  bool canRunDb = false;
  //Install RecvDB  Only on CCH channel.
  if(m_interfaceExclusions.find (m_CCHinterface) == m_interfaceExclusions.end ())
    {
      // Create a socket to listen only on this interface
      Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (),
                                                 UdpSocketFactory::GetTypeId ());
      // TRUE
      socket->SetAllowBroadcast (true);
      InetSocketAddress
        inetAddr (m_ipv4->GetAddress (m_CCHinterface, 0).GetLocal (), DB_PORT_NUMBER);
      socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvDB,  this));
      if (socket->Bind (inetAddr))
        {
          NS_FATAL_ERROR ("Failed to bind() DB socket");
        }
      socket->BindToNetDevice (m_ipv4->GetNetDevice (m_CCHinterface));
      m_socketAddresses[socket] = m_ipv4->GetAddress (m_CCHinterface, 0);//m_socketAddresses only for CCH
                                                                         //because sendpacket via CCH

      canRunDb = true;
    }

  Init_NumArea();
  if(canRunDb)
    {
      HelloTimerExpire ();
      RmTimerExpire ();
      //APTimerExpire ();
      NS_LOG_DEBUG ("DB on node (Car) " << m_CCHmainAddress << " started");
    }
}

void 
RoutingProtocol::SetCCHInterface (uint32_t interface)
{
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
  m_interfaceExclusions = exceptions;
}

//
// \brief Processes an incoming %DB packet (Car Side).
void
RoutingProtocol::RecvDB (Ptr<Socket> socket)
{
  //if (m_CCHmainAddress.Get () % 256 > 50)
 // std::cout<<"RecvDB"<<m_CCHmainAddress.Get () % 256<<std::endl;
  
  //std::cout << m_CCHmainAddress.Get()%256 << std::endl << std::endl; 
  Ptr<Packet> receivedPacket;
  Address sourceAddress;
  receivedPacket = socket->RecvFrom (sourceAddress);//CCH address
   
  InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom (sourceAddress);
  Ipv4Address senderIfaceAddr = inetSourceAddr.GetIpv4 ();
  
  Ipv4Address receiverIfaceAddr = m_socketAddresses[socket].GetLocal ();
  NS_ASSERT (receiverIfaceAddr != Ipv4Address ());
  NS_LOG_DEBUG ("DB node " << m_CCHmainAddress
                << " received a DB packet from "
                << senderIfaceAddr << " to " << receiverIfaceAddr);

  // All routing messages are sent from and to port RT_PORT,
  // so we check it.
  NS_ASSERT (inetSourceAddr.GetPort () == DB_PORT_NUMBER);

  Ptr<Packet> packet = receivedPacket;

  db::PacketHeader dbPacketHeader;
  packet->RemoveHeader (dbPacketHeader);
  NS_ASSERT (dbPacketHeader.GetPacketLength () >= dbPacketHeader.GetSerializedSize ());
  uint32_t sizeLeft = dbPacketHeader.GetPacketLength () - dbPacketHeader.GetSerializedSize ();

  MessageList messages;

  while (sizeLeft)
    {
      MessageHeader messageHeader;
      if (packet->RemoveHeader (messageHeader) == 0)
        NS_ASSERT (false);

      sizeLeft -= messageHeader.GetSerializedSize ();

      NS_LOG_DEBUG ("DB Msg received with type "
                    << std::dec << int (messageHeader.GetMessageType ())
                    << " TTL=" << int (messageHeader.GetTimeToLive ())
                    << " SeqNum=" << messageHeader.GetMessageSequenceNumber ());
      messages.push_back (messageHeader);
      
    }

  m_rxPacketTrace (dbPacketHeader, messages);
  
  for (MessageList::const_iterator messageIter = messages.begin ();
       messageIter != messages.end (); ++messageIter)
    {
      std::cout   << m_SCHmainAddress <<" RecvDB IP =" << messageIter->GetHello().ID << " "<<messageIter->GetHello().milliSeconds <<std::endl << std::endl;
      const MessageHeader &messageHeader = *messageIter;
      // If ttl is less than or equal to zero, or
      // the receiver is the same as the originator,
      // the message must be silently dropped
      //if ((messageHeader.GetTimeToLive () == 0)||(IsMyOwnAddress (dbPacketHeader.originator)))
      if ((messageHeader.GetTimeToLive () == 0)||(messageHeader.GetOriginatorAddress () == m_CCHmainAddress))
        {
          // ignore it
          packet->RemoveAtStart (messageHeader.GetSerializedSize () - messageHeader.GetSerializedSize () );
          continue;
        }

      //std::cout<<"preprocesshm " << messageHeader.GetOriginatorAddress().Get ()<<std::endl;

      switch (messageHeader.GetMessageType ())
        {
        case db::MessageHeader::ROUTING_MESSAGE:
          NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                        << "s DB node " << m_CCHmainAddress
                        << " received Routing message of size " 
                        << messageHeader.GetSerializedSize ());
          //Controller Node should discare Hello_Message
          if (GetType() == CAR  )
          {
        	  if(m_mobility->GetPosition().x<=1000.0 && senderIfaceAddr.Get()%256 == 81)//todo
                  ProcessRm (messageHeader);
        	  else if(m_mobility->GetPosition().x>1000.0 && senderIfaceAddr.Get()%256 == 84)
        		  ProcessRm (messageHeader);
          }
          break;

        case db::MessageHeader::HELLO_MESSAGE:
          NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                        << "s DB node " << m_CCHmainAddress
                        << " received Routing message of size "
                        << messageHeader.GetSerializedSize ());
          //Car Node should discare Hello_Message
          if (GetType() == LOCAL_CONTROLLER)
          {
            ProcessHM (messageHeader,senderIfaceAddr);
          }
          break;

        case db::MessageHeader::APPOINTMENT_MESSAGE:
          NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                        << "s DB node " << m_CCHmainAddress
                        << " received Appointment message of size "
                        << messageHeader.GetSerializedSize ());
          if (GetType() == CAR)
            ProcessAppointment (messageHeader);
          break;
        case db::MessageHeader::CARROUTEREQUEST_MESSAGE:
          NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                        << "s DB node " << m_CCHmainAddress
                        << " received CRREQ message of size "
                        << messageHeader.GetSerializedSize ());
          if (GetType() == LOCAL_CONTROLLER)
            ProcessCRREQ (messageHeader);
          break;
        case db::MessageHeader::CARROUTERESPONCE_MESSAGE:
          NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                        << "s DB node " << m_CCHmainAddress
                        << " received CRREP message of size "
                        << messageHeader.GetSerializedSize ());
          if (GetType() == LOCAL_CONTROLLER)
            ProcessCRREP (messageHeader);
          break;
        default:
          NS_LOG_DEBUG ("DB message type " <<
                        int (messageHeader.GetMessageType ()) <<
                        " not implemented");
        }

    }
    
}// End of RecvDB

void
RoutingProtocol::ProcessHM (const db::MessageHeader &msg,const Ipv4Address &senderIface)
{
  // LC才会运行这个函数

  /*std::cout<<m_CCHmainAddress.Get ()%256<<" RoutingProtocol::ProcessHM "
      <<msg.GetHello ().ID.Get ()%256<<" m_lc_info size:"
      <<m_lc_info.size ()<<std::endl;
  */
  Ipv4Address ID = msg.GetHello().ID;//should be SCH address
  /*if(IP2Pos.find(ID) == IP2Pos.end() )
	  IP2Pos.insert(std::map<Ipv4Address,Vector3D>::value_type(ID,msg.GetHello ().GetPosition ()));
  else
	  IP2Pos.find(ID)->second = msg.GetHello ().GetPosition ();*/
  std::cout << "Get a HelloPacket  From IP = " << ID << std::endl;
  //std::cout << msg.GetHello().GetPosition().x << " " << msg.GetHello().GetPosition().y << std::endl;
  std::cout << "LC IP = "<<m_CCHmainAddress.Get()%256 << std::endl << std::endl; 
  //dis2Ip.insert(std::map<double,Ipv4Address>::value_type((cit->second.GetPos().x-lcPosition.x),cit->first));

  /*m_SCHaddr2CCHaddr[ID] = msg.GetOriginatorAddress();
  //m_SCHaddr2IfaceAddr[ID] = senderIface;
  //std::cout<<"ProcessHM " << msg.GetOriginatorAddress().Get ()<<std::endl;
  //if(m_CCHmainAddress.Get()%256==244 )
	 // std::cout<<"244ProcessHM " << msg.GetHello ().GetPosition ().x<<std::endl;
  if(m_CCHmainAddress.Get()%256==81 && msg.GetHello ().GetPosition ().x>1000.0)//todo
	  return;
  if(m_CCHmainAddress.Get()%256==84 && msg.GetHello ().GetPosition ().x<=1000.0)
 	  return;
  std::map<Ipv4Address, CarInfo>::iterator it = m_lc_info.find (ID);
  //if(m_CCHmainAddress.Get()%256==244 )
  	 // std::cout<<"！ " << msg.GetHello ().GetPosition ().x<<std::endl;
  if (it != m_lc_info.end ())
    {
      it->second.Active = true;
      it->second.LastActive = Simulator::Now ();
      it->second.Position = msg.GetHello ().GetPosition ();
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
      m_lc_info[ID] = CI_temp;
    }*/
}

// \brief Build routing table according to Rm
void
RoutingProtocol::ProcessRm (const db::MessageHeader &msg)
{
  NS_LOG_FUNCTION (msg);
  
  const db::MessageHeader::Rm &rm = msg.GetRm();
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
      for (std::vector<db::MessageHeader::Rm::Routing_Tuple>::const_iterator it = rm.routingTables.begin();
            it != rm.routingTables.end();
            ++it)
      {
        //std::cout<<"9999 "<<rm.ID.Get ()<<" "<<rm.ID.Get ()%256<<" "<<it->destAddress.Get ()<<" "<<it->destAddress.Get ()%256<<" "<<it->nextHop.Get ()<<" "<<it->nextHop.Get ()%256<<std::endl;
        AddEntry(it->destAddress,
                 it->mask,
                 it->nextHop,
                 m_SCHinterface);
      }
    }
}

void
RoutingProtocol::ProcessAppointment (const db::MessageHeader &msg)
{
  NS_LOG_FUNCTION (msg);
  const db::MessageHeader::Appointment &appointment = msg.GetAppointment ();
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
RoutingProtocol::ProcessCRREQ (const db::MessageHeader &msg)
{
  NS_LOG_FUNCTION (msg);
  const db::MessageHeader::CRREQ &crreq = msg.GetCRREQ ();
  Ipv4Address dest =  crreq.destAddress;
  Ipv4Address source = crreq.sourceAddress;//the car's ip address
  if(m_CCHmainAddress.Get()%256 == 81)// need to expand//todo
	  return;
  if(m_lc_info.find(dest)==m_lc_info.end() || m_lc_info.size()<=1)
	  return;
  /*for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
       cit != m_lc_info.end (); ++cit)
    {
	  std::cout<<"info"<<cit->first.Get()%256<<std::endl;
    }*/
  //std::cout<<"ProcessCRREQ"<<transferAddress.Get()%256<<std::endl;
  if(transferAddress == dest)
	  return;
  SendCRREP(source, dest, transferAddress);
}

void
RoutingProtocol::ProcessCRREP (const db::MessageHeader &msg)
{
  NS_LOG_FUNCTION (msg);
  const db::MessageHeader::CRREP &crrep = msg.GetCRREP ();
  Ipv4Address dest =  crrep.destAddress;
  Ipv4Address source = crrep.sourceAddress;
  Ipv4Address transfer = crrep.transferAddress;
 //std::cout<<"ProcessCRREP"<<transfer.Get()%256<<" "<<dest.Get()%256<<std::endl;
  if(m_CCHmainAddress.Get()%256 == 84)
	  return;
  Ipv4Address mask("255.255.255.0");
  //ComputeRoute();
  LCAddEntry(roadendAddress,dest,mask,transfer);
  //std::cout<<"roadendAddress"<<roadendAddress.Get()%256<<std::endl;
 // std::cout<<"infosize"<<m_lc_info.size()<<std::endl;
  //std::cout<<"roadendAddress"<<roadendAddress.Get()%256<<std::endl;
  int t=0;
  for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
       cit != m_lc_info.end (); ++cit)
    {
	  CarInfo Entry = cit->second;//std::cout<<t<<"ProcessCRREP1"<<Entry.R_Table.size()<<std::endl;
	  t++;
	  for(std::vector<RoutingTableEntry>::iterator it=Entry.R_Table.begin();it!=Entry.R_Table.end();++it)
	  {
		  //std::cout<<"table"<<cit->first.Get()%256<<" "<<it->destAddr.Get()%256<<" "<<it->nextHop.Get()%256<<std::endl;
	      if(it->destAddr == roadendAddress)
	      {
	    	  /*RoutingTableEntry RTE;
	    	  RTE.destAddr = dest;
	    	  RTE.mask = it->mask;
	    	  RTE.nextHop = it->nextHop;
	    	  std::cout<<"entry"<<cit->first.Get()%256<<" "<<dest.Get()%256<<" "<<it->nextHop.Get()%256<<std::endl;
	    	  RTE.interface = it->interface;
	    	  Entry.R_Table.push_back(RTE);*/
	    	  LCAddEntry(cit->first,dest,it->mask,it->nextHop);
	    	  break;
	      }
	  }

    }
  /*for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
       cit != m_lc_info.end (); ++cit)
  {
	  CarInfo Entry = cit->second;
	  for(std::vector<RoutingTableEntry>::iterator it=Entry.R_Table.begin();it!=Entry.R_Table.end();++it)
	  {
		  std::cout<<"table"<<cit->first.Get()%256<<" "<<it->destAddr.Get()%256<<" "<<it->nextHop.Get()%256<<std::endl;
	  }
  }*/
  //std::cout<<"ProcessCRREP"<<std::endl;
  	 SendRoutingMessage();
}

void
RoutingProtocol::Clear()
{
  NS_LOG_FUNCTION_NOARGS();
  m_table.clear();
  
}

void
RoutingProtocol::AddEntry (const Ipv4Address &dest,
                           const Ipv4Address &mask,
                           const Ipv4Address &next,
                           uint32_t interface)
{
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
  NS_LOG_FUNCTION (this << " " << m_ipv4->GetObject<Node> ()->GetId () << " " << header.GetDestination ());
  // if(header.GetDestination ().Get ()%256 != 255)
   //std::cout<<"RouteInput "<<header.GetSource ().Get ()%256<< ", "<<m_SCHmainAddress.Get () %256<< ",Dest:"<<header.GetDestination ().Get ()%256<<std::endl;
  //bool lcb_status = false;
  Ipv4Address dest = header.GetDestination();
  Ipv4Address sour = header.GetSource();
  int iden=header.GetIdentification();
  //std::cout<<"iden:"<<iden<<std::endl;
  if(find(hasbeensend.begin(),hasbeensend.end(),iden) !=hasbeensend.end())
  	  return true;
  hasbeensend.push_back(iden);
  double distance = 0;
  Vector3D thispos = m_mobility->GetPosition ();
  std::map<Ipv4Address, Vector3D>::const_iterator iiit = IP2Pos.find(sour);
  if(iiit!= IP2Pos.end()  && ( iiit->second.x >  thispos.x) )
	  distance = CalculateDistance(iiit->second,thispos);
  // Consume self-originated packets
  if (IsMyOwnAddress (sour) == true)
    {
      return true;
    }

  if (header.GetTtl () == 0)//avoid broadcast storm
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
  //Broadcast forwardfind(numBitmapIp.begin(),numBitmapIp.end(),citi->second)-

  if ((iif == m_SCHinterface) && (m_nodetype == CAR)  )//todo to do sth to guarantee forward once
               {
                 NS_LOG_LOGIC ("Forward broadcast");
                 Ptr<Ipv4Route> broadcastRoute = Create<Ipv4Route> ();
                // broadcastRoute->SetGateway (dest);//broadcast
                 broadcastRoute->SetGateway(Ipv4Address::GetBroadcast());
                 broadcastRoute->SetOutputDevice (m_ipv4->GetNetDevice (m_SCHinterface));
                 Ipv4Header ipHeader = header;
                 ipHeader.SetSource (m_ipv4->GetAddress (m_SCHinterface,0).GetLocal ()); //To Prevent Brocast Storm, m_CCHmainAddress is for CCH.
                 ipHeader.SetTtl (ipHeader.GetTtl () - 1);
                 if (ipHeader.GetTtl ()!=0)
                   {
                	 Time sendInterval;
                	 if(distance>0)
                	 {
                		 sendInterval=Seconds(1/(distance*100));
                		 Simulator::Schedule(sendInterval, &RoutingProtocol::Senducb, this, ucb, broadcastRoute, p, ipHeader);
                	 }
                	 else
                		 	ucb (broadcastRoute, p, ipHeader);
                    // std::cout<<"FORWARD,UCB,TTL:"<<int(ipHeader.GetTtl())<<std::endl;
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
      
      /*//Forwardding
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
              NS_FATAL_ERROR ("XXX Not implemented yet:  IP aliasing and DB");
          }
          rtentry->SetSource (ifAddr.GetLocal ());
          rtentry->SetGateway (entry.nextHop);
          rtentry->SetOutputDevice (m_ipv4->GetNetDevice (interfaceIdx));
          NS_LOG_DEBUG ("DB node " << m_CCHmainAddress
                                 << ": RouteInput for dest=" << header.GetDestination ()
                                 << " --> nextHop=" << entry.nextHop
                                 << " interface=" << entry.interface);
          NS_LOG_DEBUG ("Found route to " << rtentry->GetDestination () << " via nh " << rtentry->GetGateway () << " with source addr " << rtentry->GetSource () << " and output dev " << rtentry->GetOutputDevice ());
          ucb (rtentry, p, header);
      }
      else
      {
          NS_LOG_DEBUG ("DB node " << m_CCHmainAddress
                                 << ": RouteInput for dest=" << header.GetDestination ()
                                 << " No route to host");
          //std::cout<<"2No route to host"<<std::endl;
      }*/
        
      return true;

    /*}
  //Drop
  return true;*/
}
void RoutingProtocol::Senducb (UnicastForwardCallback ucb,Ptr<Ipv4Route> broadcastRoute,Ptr<const Packet> p,Ipv4Header ipHeader)
{
	ucb (broadcastRoute, p, ipHeader);
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
  NS_LOG_FUNCTION (this << " " << m_ipv4->GetObject<Node> ()->GetId () << " " << header.GetDestination () << " " << oif);
  Ptr<Ipv4Route> rtentry;
  RoutingTableEntry entry;
  //std::cout<<"RouteOutput "<<m_SCHmainAddress.Get () << ",Dest:"<<header.GetDestination ().Get ()<<std::endl;
  //std::cout<<"M_TABLE SIZE "<<m_table.size ()<<std::endl;
  //SetCCHInterface(m_CCHinterface);
  //SetSCHInterface(m_SCHinterface);
  if (Lookup (header.GetDestination (), entry))
    {
      //std::cout<<"0found!"<<entry.nextHop.Get()%256<<std::endl;
      uint32_t interfaceIdx = entry.interface;
      if (oif && m_ipv4->GetInterfaceForDevice (oif) != static_cast<int> (interfaceIdx))
        {
          // We do not attempt to perform a constrained routing searchTx_Data_Pkts
          // if the caller specifies the oif; we just enforce that
          // that the found route matches the requested outbound interface
          NS_LOG_DEBUG ("DB node " << m_SCHmainAddress
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
          NS_FATAL_ERROR ("XXX Not implemented yet:  IP aliasing and DB");
        }
      rtentry->SetSource (ifAddr.GetLocal ());
      rtentry->SetGateway (entry.nextHop);
      rtentry->SetOutputDevice (m_ipv4->GetNetDevice (interfaceIdx));
      sockerr = Socket::ERROR_NOTERROR;
      //std::cout<<"***"<<rtentry->GetDestination ().Get()<<" "<<rtentry->GetGateway ().Get()<<std::endl;
      NS_LOG_DEBUG ("DB node " << m_SCHmainAddress
                                 << ": RouteOutput for dest=" << header.GetDestination ()
                                 << " --> nextHop=" << entry.nextHop
                                 << " interface=" << entry.interface);
      NS_LOG_DEBUG ("Found route to " << rtentry->GetDestination () << " via nh " << rtentry->GetGateway () << " with source addr " << rtentry->GetSource () << " and output dev " << rtentry->GetOutputDevice ());
    }
  else
    {
      NS_LOG_DEBUG ("DB node " << m_SCHmainAddress
                                 << ": RouteOutput for dest=" << header.GetDestination ()
                                 << " No route to host");
      sockerr = Socket::ERROR_NOROUTETOHOST;
      //SendCRREQ(header.GetDestination());
      //std::cout<<"No route to host"<<std::endl;
      uint32_t interfaceIdx = m_SCHinterface;
      Ipv4InterfaceAddress ifAddr;
      ifAddr = m_ipv4->GetAddress (interfaceIdx, 0);
      rtentry = Create<Ipv4Route> ();
      rtentry->SetSource (ifAddr.GetLocal ());
     rtentry->SetGateway (Ipv4Address::GetBroadcast());
      //rtentry->SetGateway (Ipv4Address::GetZero());
      rtentry->SetOutputDevice (m_ipv4->GetNetDevice (interfaceIdx));
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
  m_packetSequenceNumber = (m_packetSequenceNumber + 1) % (DB_MAX_SEQ_NUM + 1);
  return m_packetSequenceNumber;
}


uint16_t
RoutingProtocol::GetMessageSequenceNumber ()
{
  m_messageSequenceNumber = (m_messageSequenceNumber + 1) % (DB_MAX_SEQ_NUM + 1);
  return m_messageSequenceNumber;
}

void
RoutingProtocol::HelloTimerExpire ()
{
  //std::cout<<"HmTimerExpire "<<m_CCHmainAddress.Get ()%256;
  //std::cout<<", Time:"<<Simulator::Now().GetSeconds ()<<std::endl;
  if (GetType() == CAR)
    {
      //std::cout << "~~ininin~~" << std::endl;
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

 /*if (GetType () == LOCAL_CONTROLLER)
  {
      ClearAllTables();//std::cout<<"1:"<<std::endl;
      ComputeRoute ();//std::cout<<"2:"<<std::endl;
      SendRoutingMessage ();//std::cout<<"3:"<<std::endl;
      m_rmTimer.Schedule (m_rmInterval);//std::cout<<"4:"<<std::endl;
  }*/
}

void
RoutingProtocol::APTimerExpire ()
{
  /*if (GetType() == LOCAL_CONTROLLER)
    {
      ComputeRoute ();
    }*/
}


// DB packets actually send here.
void
RoutingProtocol::SendPacket (Ptr<Packet> packet,
                             const MessageList &containedMessages)
{
  NS_LOG_DEBUG ("DB node " << m_CCHmainAddress << " sending a DB packet");
  //std::cout<<"DB node " << m_CCHmainAddress.Get ()<< " sending a DB packet"<<std::endl;
  // Add a header
  db::PacketHeader header;
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
      i->first->SendTo (packet, 0, InetSocketAddress (bcast, DB_PORT_NUMBER));
    }
}

void
RoutingProtocol::QueueMessage (const db::MessageHeader &message, Time delay)
{
   m_queuedMessages.push_back (message);
  if (not m_queuedMessagesTimer.IsRunning ())
    {
      m_queuedMessagesTimer.SetDelay (delay);
      m_queuedMessagesTimer.Schedule ();
    }
}


// NS3 is not multithread, so mutex is unnecessary.
// Here, messages will queue up and send once numMessage is equl to DB_MAX_MSGS.
// This function will NOT add a header to each message
void
RoutingProtocol::SendQueuedMessages ()
{
  Ptr<Packet> packet = Create<Packet> ();
  int numMessages = 0;

  NS_LOG_DEBUG ("DB node " << m_CCHmainAddress << ": SendQueuedMessages");
  //std::cout<<"SendQueuedMessages  "<<m_CCHmainAddress.Get ()%256 <<std::endl;
  MessageList msglist;

  for (std::vector<db::MessageHeader>::const_iterator message = m_queuedMessages.begin ();
       message != m_queuedMessages.end ();
       ++message)
    {
      Ptr<Packet> p = Create<Packet> ();
      p->AddHeader (*message);
      packet->AddAtEnd (p);
      msglist.push_back (*message);
      if (++numMessages == DB_MAX_MSGS)
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
  NS_LOG_FUNCTION (this);
  db::MessageHeader msg;
   //ns3/src/core/model/timer.h
  msg.SetVTime (m_helloInterval);
  msg.SetTimeToLive (41993);//Just MY Birthday.
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  msg.SetMessageType (db::MessageHeader::HELLO_MESSAGE);
  msg.SetOriginatorAddress(m_CCHmainAddress);
 // std::cout<<"SendHello " <<m_mobility->GetPosition ().x<<std::endl;
  db::MessageHeader::Hello &hello = msg.GetHello ();
  hello.ID = m_SCHmainAddress;
  Time now = Simulator::Now ();
  hello.milliSeconds = 10;
  std::cout << hello.milliSeconds;
  Vector pos = m_mobility->GetPosition ();
  Vector vel = m_mobility->GetVelocity ();
  hello.SetPosition (pos.x, pos.y, pos.z);
  hello.SetVelocity (vel.x, vel.y, vel.z);
  std::cout << " time:" << now.GetMilliSeconds() << ":" << now.GetMicroSeconds() << " location:" << pos.x << " " << pos.y << " Send a HelloPacket Ip = " << m_SCHmainAddress << std::endl << std::endl;
  //std::cout << pos.x << " " << pos.y  << " " << pos.z << std::endl;
  //std::cout << vel.x << " " << vel.y  << " " << vel.z << std::endl << std::endl;
  NS_LOG_DEBUG ( "DB HELLO_MESSAGE sent by node: " << hello.ID
                 << "   at " << now.GetSeconds() << "s");
  QueueMessage (msg, JITTER);
}

void
RoutingProtocol::SendRoutingMessage ()
{
  NS_LOG_FUNCTION (this);
  //std::cout<<"SendRoutingMessage"<<m_CCHmainAddress.Get()%256<<std::endl;
  for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
       cit != m_lc_info.end (); ++cit)
    {
      db::MessageHeader msg;
      Time now = Simulator::Now ();
      msg.SetVTime (m_helloInterval);
      msg.SetTimeToLive (41993);//Just MY Birthday.
      msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
      msg.SetMessageType (db::MessageHeader::ROUTING_MESSAGE);
      msg.SetOriginatorAddress(m_CCHmainAddress);
      db::MessageHeader::Rm &rm = msg.GetRm ();
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
      db::MessageHeader::Rm::Routing_Tuple rt;
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
  NS_LOG_FUNCTION (this);

  for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
       cit != m_lc_info.end (); ++cit)
    {
      db::MessageHeader msg;
      Time now = Simulator::Now ();
      msg.SetVTime (m_helloInterval);
      msg.SetTimeToLive (41993);//Just MY Birthday.
      msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
      msg.SetMessageType (db::MessageHeader::APPOINTMENT_MESSAGE);
      db::MessageHeader::Appointment &appointment = msg.GetAppointment ();
      appointment.ID = cit->first;
      appointment.ATField = cit->second.appointmentResult;
      appointment.NextForwarder = cit->second.ID_of_minhop;
      QueueMessage (msg, JITTER);
    }
}

void
RoutingProtocol::SendCRREQ (Ipv4Address const &destAddress)
{
  NS_LOG_FUNCTION (this);

 // std::cout<<"SendCRREQ "<<std::endl;
  db::MessageHeader msg;
  Time now = Simulator::Now ();
  msg.SetVTime (m_helloInterval);
  msg.SetTimeToLive (41993);//Just MY Birthday.
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  msg.SetMessageType (db::MessageHeader::CARROUTEREQUEST_MESSAGE);
  db::MessageHeader::CRREQ &crreq = msg.GetCRREQ ();
  crreq.sourceAddress=m_CCHmainAddress;
  crreq.destAddress=destAddress;
  QueueMessage (msg, JITTER);
}

void
RoutingProtocol::SendCRREP( Ipv4Address const &sourceAddress,
		Ipv4Address const&destAddress, Ipv4Address const &transferAddress)
{
  NS_LOG_FUNCTION (this);

  //std::cout<<"SendCRREP "<<std::endl;
  db::MessageHeader msg;
  Time now = Simulator::Now ();
  msg.SetVTime (m_helloInterval);
  msg.SetTimeToLive (41993);//Just MY Birthday.
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  msg.SetMessageType (db::MessageHeader::CARROUTERESPONCE_MESSAGE);
  db::MessageHeader::CRREP &crrep = msg.GetCRREP ();
  crrep.sourceAddress=sourceAddress;
  crrep.destAddress=destAddress;
  crrep.transferAddress=transferAddress;
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
/*std::cout<<"RemoveTimeOut"<<std::endl;
    RemoveTimeOut (); //Remove Stale Tuple

    if (!m_linkEstablished)
      {
        std::cout<<"Do_Init_Compute"<<std::endl;
        Do_Init_Compute ();
      }
    else
      {
        std::cout<<"Do_Update"<<std::endl;
        Do_Update ();

      }

    std::cout<<"SendAppointment"<<std::endl;
    SendAppointment ();
    Reschedule ();
    std::cout<<"CR DONE"<<std::endl;*/
    //Remove Timeout Tuples first.
    //std::cout<<"RemoveTimeOut"<<m_CCHmainAddress.Get()%256<<std::endl;
    RemoveTimeOut (); //Remove Stale Tuple
    //input the m_lc_info;

    //turn m_lc_info to dis2Ip(sord by dis) and numBitmapIp(Bitmap the num and the Ip)
    //Vector3D lcPosition(0,0,0);//the last parameter can be angle
    Vector3D lcPosition = m_mobility->GetPosition ();
    std::map<double,Ipv4Address> dis2Ip;
    std::vector<Ipv4Address> numBitmapIp;

    for (std::map<Ipv4Address,CarInfo>::iterator cit = m_lc_info.begin (); cit != m_lc_info.end (); ++cit)
    {
        //cout<<CalculateDistance(cit->second.GetPos(),lcPosition)<<endl;
        dis2Ip.insert(std::map<double,Ipv4Address>::value_type((cit->second.GetPos().x-lcPosition.x),cit->first));
       // if(cit->first.Get()%256 == 30)
        	//std::cout<<" 30x "<<cit->second.GetPos().x<<std::endl;
    }

    numBitmapIp.clear();
    for (std::map<double,Ipv4Address>::iterator cit = dis2Ip.begin (); cit != dis2Ip.end (); ++cit)
    {
        numBitmapIp.push_back(cit->second);
        if(cit->first<=900)
         roadendAddress = cit->second;
        //cout<<cit->second.m_address<<endl;
    }
    if(dis2Ip.size()>=1)//because it will compute once before everything start and size can be 0
    {
        transferAddress = dis2Ip.begin()->second;
        //std::cout<<"ku"<<m_CCHmainAddress.Get()%256<<roadendAddress.Get()%256<<" "<<transferAddress.Get()%256<<std::endl;
    }
    //if(numBitmapIp.size()>1)//because it will compute once before everything start and size can be 0
    //	std::cout<<numBitmapIp.size()<<"?????????"<<std::endl;
    //build the topology graph.
    Edge edge[max_car_number*max_car_number];     // 保存边的值
    //memset(edge,max_car_number,sizeof(edge));
    for(int t=0;t<max_car_number*max_car_number;t++)
    {
        edge[t].weight=MAX;
    }
    int k=0;
    for(std::map<double,Ipv4Address>::iterator citi = dis2Ip.begin(); citi != dis2Ip.end(); ++citi)
    {
        for(std::map<double,Ipv4Address>::iterator citj = citi; citj != dis2Ip.end(); ++citj)
        {
            if(citi != citj)
            {
                if(abs(citj->first - citi->first)<m_signal_range)
                {
                    edge[k].u=find(numBitmapIp.begin(),numBitmapIp.end(),citi->second)-numBitmapIp.begin();//Ipv4Address need to overwrite ==
                    edge[k].v=find(numBitmapIp.begin(),numBitmapIp.end(),citj->second)-numBitmapIp.begin();
                    edge[k].weight=1;
                    //std::cout<<numBitmapIp[edge[k].u].Get()%256<<" "<<numBitmapIp[edge[k].v].Get()%256<<std::endl;
                    ++k;
                    //cout<<find(numBitmapIp.begin(),numBitmapIp.end(),citj->second)-numBitmapIp.begin()<<endl;
                }
            }
        }
    }
    //std::cout<<k<<" 123"<<std::endl;

//...........
    //calculate the shortest distance using bellman-ford algorithm  -- todo
    int listsize=numBitmapIp.size();
    for(int i=0; i<listsize; i++)
    {
        //Bellman_Ford();
        //define
        int dist[max_car_number];
        int pre[max_car_number];
        //initialzing;
        int nodenum=listsize;
        int edgenum=k;

        //memset(dist,MAX,max_car_number);
        for(int t=0;t<max_car_number;t++)
                dist[t]=MAX;
        dist[i]=0;
        //memset(pre,max_car_number,sizeof(pre));
        //pre[max_car_number-1]=max_car_number-1;//xiao xin yi chu!!!!!
        /*for(int t=0;t<max_car_number;t++)
        	std::cout<<dist[t]<<"?.?"<<std::endl;*/
        for(int t=0; t<max_car_number;t++)
                pre[t]=t;

        //relax:
        for(int t=0; t<nodenum-1; ++t)
        {
            for(int j=0; j<edgenum; ++j)
            {
                //std::cout<<dist[edge[j].v]<<" "<<dist[edge[j].u]<<" "<<numBitmapIp[t].Get()%256<<" "<<edge[j].weight<<"!!!!!"<<std::endl;
                if(dist[edge[j].v] > dist[edge[j].u] + edge[j].weight)
                {
                    dist[edge[j].v] = dist[edge[j].u] + edge[j].weight;
                    pre[edge[j].v]=edge[j].u;
                    //std::cout<<"8888 "<<edge[j].u<<" "<<edge[j].v<<std::endl;
                }
            }
        }
        /*for(int t=0;t<listsize;t++)
        {
        	std::cout<<pre[t]<<std::endl;
        }*/

        bool flag = 1;
        // 判断是否有负环路
        if(edgenum!=0)
			for(int t=1; t<=edgenum; ++t)
			{

				if(dist[edge[t].v] > dist[edge[t].u] + edge[t].weight)
				{
					//std::cout<<edge[t].v<<":"<<dist[edge[t].v]<<" "<<edge[t].u<<":"<<dist[edge[t].u]<<" "<<t<<"?????"<<std::endl;
					flag = 0;
					break;
				}
			}

        if(!flag)
        {
            std::cout<<"ERROR: There is a negative weight edge in the VANETs!!"<<std::endl;
        }

        //record the route
        //Ipv4Address bcast = Ipv4Address::GetBroadcast();
        //Ipv4Address allzero = Ipv4Address::GetZero ();
        Ipv4Address mask("255.255.255.0");
        //std::cout<<i<<" "<<listsize<<" 77777"<<std::endl;
        for(int t=1; t<listsize; t++)
        {
            int root = t;
            while(root != pre[root])
            {
            	//if(m_CCHmainAddress.Get()%256==244)
                   //std::cout<<pre[root]<<":"<<numBitmapIp[pre[root]].Get()<<"->"<<t<<":"<<numBitmapIp[t].Get()<<"crossing"<<root<<":"<<numBitmapIp[root].Get()<<std::endl;
                LCAddEntry (numBitmapIp[pre[root]], numBitmapIp[t], mask, numBitmapIp[root]);//todo
                root = pre [root];
            }
        }
    }
    /*for (map<Ipv4Address,CarInfo>::iterator cit = m_lc_info.begin (); cit != m_lc_info.end (); ++cit)
    {
        cout<<cit->first.m_address<<" "<<cit->second.Position.x<<" "<<cit->second.Position.y<<" "<<cit->second.Position.z<<endl;
    }*/

    std::cout << "Hello world!" << std::endl;
}//RoutingProtocol::ComputeRoute

void
RoutingProtocol::Do_Init_Compute ()
{
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
  int numArea = GetNumArea();
  for (std::set<Ipv4Address>::const_iterator cit = m_Sections[numArea-1].begin ();
      cit != m_Sections[numArea-1].end (); ++cit)
    {
      m_lc_info[(*cit)].minhop = 1;
      m_lc_info[(*cit)].ID_of_minhop = Ipv4Address::GetZero ();
    }
}

void
RoutingProtocol::OtherSet_Init ()
{
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
  for (std::map<Ipv4Address, CarInfo>::iterator it = m_lc_info.begin ();
       it != m_lc_info.end (); ++it)
    {
      it->second.appointmentResult = NORMAL;
    }
}

void
RoutingProtocol::ShiftArea ()
{
  for (int i = GetNumArea () - 1; i>0; --i)
    {
      m_Sections[i] = m_Sections[i-1];
    }
  m_Sections[0].clear ();
}

void
RoutingProtocol::AddNewToZero ()
{
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
  NS_LOG_FUNCTION(this  << ID << dest << next << interface << mask << m_CCHmainAddress);
  //std::cout<<"dest:"<<m_next_forwarder.Get () % 256<<std::endl;
  CarInfo& Entry = m_lc_info[ID];std::cout<<"Interfaces:"<<std::endl;
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
  Entry.R_Table.push_back (RTE);
}

void
RoutingProtocol::LCAddEntry (const Ipv4Address& ID,
                           const Ipv4Address &dest,
                           const Ipv4Address &mask,
                           const Ipv4Address &next,
                           const Ipv4Address &interfaceAddress)
{
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
  Time now = Simulator::Now ();
  std::map<Ipv4Address, CarInfo>::iterator it = m_lc_info.begin ();
  std::vector<Ipv4Address> pendding;
  while (it != m_lc_info.end ())
    {
      if (now.GetSeconds() - it->second.LastActive.GetSeconds () > 3 * m_helloInterval.GetSeconds())
        {
          pendding.push_back (it->first);
        }
      ++it;
    }
  for (std::vector<Ipv4Address>::iterator it = pendding.begin ();
      it != pendding.end(); ++it)
    {
      m_lc_info.erase((*it));
    }
}

void
RoutingProtocol::SetSignalRangeNRoadLength (double signal_range, double road_length)
{
  m_signal_range = signal_range;
  m_road_length = road_length;
}

} // namespace db
} // namespace ns3


