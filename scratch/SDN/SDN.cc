/*
 * SDN.cc
 *
 *  Created on: Oct 9, 2015
 *      Author: chl
 */
/*
  ./waf --run "SDN"
*/
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <dirent.h>//DIR*
#include "SDN.h"


NS_LOG_COMPONENT_DEFINE ("SDN");


using namespace ns3;

VanetSim::VanetSim()
{
	traceFile = "";
	logFile = "SDN.log";
	phyMode = "OfdmRate6MbpsBW10MHz";
	lossModle = "ns3::FriisPropagationLossModel";
	freq1 = 5.860e9;  //802.11p SCH CH172
	freq2 = 5.890e9;  //802.11p CCH CH178z
	txp1 = 20;  // dBm SCH
	txp2 = 20;  // CCH
	range1 = 500.0;//SCH
	range2 = 3000.0;//CCH
	packetSize = 1000; // bytes
	numPackets = 1;
	interval = 0.1; // seconds
	verbose = false;
	mod = 1;
	pmod = 0;
	duration = -1;
	nodeNum = 0;
	Rx_Data_Bytes = 0;
	Rx_Data_Pkts = 0;
	Rx_Routing_Bytes = 0;
	RX_Routing_Pkts = 0;
	Tx_Data_Bytes = 0;
	Tx_Data_Pkts = 0;
	Tx_Routing_Bytes = 0;
	TX_Routing_Pkts = 0;

	Rx_Data_Bytes2 = 0;
	Rx_Data_Pkts2 = 0;
	Rx_Routing_Bytes2 = 0;
	RX_Routing_Pkts2 = 0;
	Tx_Data_Bytes2 = 0;
	Tx_Data_Pkts2 = 0;
	Tx_Routing_Bytes2 = 0;
	TX_Routing_Pkts2 = 0;

	TTL = 0;
	m_port = 65419;
	m_port2 = 65420;
	homepath = ".";//getenv("HOME");
	folder="testData";
}

VanetSim::~VanetSim()
{
	os.close();
}

void VanetSim::Simulate(int argc, char *argv[])
{
	SetDefault();//nothing
	ParseArguments(argc, argv);
	LoadTraffic();// load the sumo information
	ConfigNode();// add the node name
	ConfigChannels();
	ConfigDevices();
	ConfigMobility();
	ConfigApp();
	ConfigTracing();
	Run();
	ProcessOutputs();
	std::cout<<std::endl;
}

void VanetSim::SetDefault()
{
	//Handle By Constructor
}

void VanetSim::ParseArguments(int argc, char *argv[])
{
	//change the variable while program running, other than when the program start 
	CommandLine cmd;
//	cmd.AddValue ("traceFile", "Ns2 movement trace file", traceFile);
//	cmd.AddValue ("nodeNum", "Number of nodes", nodeNum);
	cmd.AddValue ("duration", "Duration of Simulation", duration);
//	cmd.AddValue ("logFile", "Log file", logFile);
	cmd.AddValue ("folder", "Working Directory", folder);
	cmd.AddValue ("txp1", "TX power for SCH", txp1);
	cmd.AddValue ("txp2", "TX power for CCH", txp2);
	cmd.AddValue ("range1", "Range for SCH", range1);
	cmd.AddValue ("range2", "Range for CCH", range2);
	cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
	cmd.AddValue ("mod", "0=olsr 1=sdn(DEFAULT) 2=aodv 3=dsdv 4=dsr", mod);
	cmd.AddValue ("pmod", "0=Range(DEFAULT) 1=Other", pmod);
	cmd.AddValue ("ds", "DataSet", m_ds);
	cmd.Parse (argc,argv);

	// Fix non-unicast data rate to be the same as that of unicast
	Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
	                      StringValue (phyMode));

}

void VanetSim::LoadTraffic()
{
	if (mod==0)
	{
		std::cout<<"Mode: OLSR-N"<<std::endl;
		m_todo = "OLSR";
	}
	else if(mod==1)
	{
		std::cout<<"Mode: SDN"<<std::endl;
		m_todo = "SDN";
	}
	else if(mod==2)
	{
		std::cout<<"Mode: AODV"<<std::endl;
		m_todo = "AODV";
	}
	else if(mod==3)
	{
		std::cout<<"Mode: DSR"<<std::endl;
		m_todo = "DSR";
	}
	else if(mod==4)
	{
		std::cout<<"Mode: DSDV"<<std::endl;
		m_todo = "DSDV";
	}
	DIR* dir = NULL;
	//DIR* subdir=NULL;
	std::string temp(homepath+"/"+folder);
	if((dir = opendir(temp.data()))==NULL)
		NS_FATAL_ERROR("Cannot open input path "<<temp.data()<<", Aborted.");

	std::string sumo_net = temp + "/net.xml";

	std::string sumo_fcd = temp + "/fcd.xml";
	std::string sumo_route = temp + "/rou.xml";
	std::string output = temp + "/" + m_todo + "_" + m_ds + "_result_new.txt";

	os.open(output.data(),std::ios::out);

	ns3::vanetmobility::VANETmobilityHelper mobilityHelper;
	VMo=mobilityHelper.GetSumoMObility(sumo_net,sumo_route,sumo_fcd);

	nodeNum = VMo->GetNodeSize(); //car num 
	//std::cout << "nodeNUm=" << nodeNum << std::endl;
	os<<"Mode:  "<<m_todo<<"DataSet:  "<<m_ds<<std::endl;
}



void VanetSim::ConfigNode()
{
	m_nodes.Create(nodeNum+29);//Cars + 24Controller + 2Source + 2Sink
	//std::cout<<nodeNum<<std::endl;
	/*Only Apps Are Different Between Different kind of Nodes*/
	// Name nodes
	for (uint32_t i = 0; i < nodeNum; ++i)
	{
		std::ostringstream os;
		os << "vehicle-"  << i;
 		Names::Add(os.str(), m_nodes.Get(i));
	}
	Names::Add("Controller_1",m_nodes.Get(nodeNum));
	Names::Add("Controller_2",m_nodes.Get(nodeNum+1));
	Names::Add("Controller_3",m_nodes.Get(nodeNum+2));
	Names::Add("Controller_4",m_nodes.Get(nodeNum+3));
	Names::Add("Controller_5",m_nodes.Get(nodeNum+4));
	Names::Add("Controller_6",m_nodes.Get(nodeNum+5));
	Names::Add("Controller_7",m_nodes.Get(nodeNum+6));
	Names::Add("Controller_8",m_nodes.Get(nodeNum+7));
	Names::Add("Controller_9",m_nodes.Get(nodeNum+8));
	Names::Add("Controller_10",m_nodes.Get(nodeNum+9));
	Names::Add("Controller_11",m_nodes.Get(nodeNum+10));
	Names::Add("Controller_12",m_nodes.Get(nodeNum+11));
	Names::Add("Controller_13",m_nodes.Get(nodeNum+12));
	Names::Add("Controller_14",m_nodes.Get(nodeNum+13));
	Names::Add("Controller_15",m_nodes.Get(nodeNum+14));
	Names::Add("Controller_16",m_nodes.Get(nodeNum+15));
	Names::Add("Controller_17",m_nodes.Get(nodeNum+16));
	Names::Add("Controller_18",m_nodes.Get(nodeNum+17));
	Names::Add("Controller_19",m_nodes.Get(nodeNum+18));
	Names::Add("Controller_20",m_nodes.Get(nodeNum+19));
	Names::Add("Controller_21",m_nodes.Get(nodeNum+20));
	Names::Add("Controller_22",m_nodes.Get(nodeNum+21));
	Names::Add("Controller_23",m_nodes.Get(nodeNum+22));
	Names::Add("Controller_24",m_nodes.Get(nodeNum+23));
	Names::Add("Source",m_nodes.Get(nodeNum+24));
	Names::Add("Sink",m_nodes.Get(nodeNum+25));
	Names::Add("Source2",m_nodes.Get(nodeNum+26));
	Names::Add("Sink2",m_nodes.Get(nodeNum+27));
	Names::Add("GC",m_nodes.Get(nodeNum+28));
}

void VanetSim::ConfigChannels()
{
	//===channel
	std::cout<<"ConfigChannels"<<std::endl;
	YansWifiChannelHelper SCHChannel;
	SCHChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	if (pmod == 1)
	{
		SCHChannel.AddPropagationLoss(lossModle,"Frequency", DoubleValue(freq1));
	}
	else
	{
		SCHChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange",
				DoubleValue(range1));
	}
	YansWifiChannelHelper CCHChannel;
	CCHChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	if (pmod ==1)
	{
		CCHChannel.AddPropagationLoss(lossModle,"Frequency", DoubleValue(freq2));
	}
	else
	{
		CCHChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange",
				DoubleValue(range2));
	}



	// the channelg
	Ptr<YansWifiChannel> SCH = SCHChannel.Create();
	Ptr<YansWifiChannel> CCH = CCHChannel.Create();

	//===wifiphy
	YansWifiPhyHelper SCHPhy =  YansWifiPhyHelper::Default ();
	SCHPhy.SetChannel (SCH);
	SCHPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
	YansWifiPhyHelper CCHPhy =  YansWifiPhyHelper::Default ();
	CCHPhy.SetChannel (CCH);
	CCHPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);

	// 802.11p mac
	NqosWaveMacHelper SCH80211pMac = NqosWaveMacHelper::Default ();
	Wifi80211pHelper SCH80211p = Wifi80211pHelper::Default ();
	NqosWaveMacHelper CCH80211pMac = NqosWaveMacHelper::Default ();
	Wifi80211pHelper CCH80211p = Wifi80211pHelper::Default ();

	SCH80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
										"DataMode",StringValue (phyMode),
										"ControlMode",StringValue (phyMode));
	CCH80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
											"DataMode",StringValue (phyMode),
											"ControlMode",StringValue (phyMode));

	// Set Tx Power For The SCH
	SCHPhy.Set ("TxPowerStart",DoubleValue (txp1));
	SCHPhy.Set ("TxPowerEnd", DoubleValue (txp1));
	m_SCHDevices = SCH80211p.Install(SCHPhy, SCH80211pMac, m_nodes);

	// CCH
	CCHPhy.Set ("TxPowerStart",DoubleValue (txp2));
	CCHPhy.Set ("TxPowerEnd", DoubleValue (txp2));
	m_CCHDevices = CCH80211p.Install(CCHPhy, CCH80211pMac, m_nodes);

}

void VanetSim::ConfigDevices()
{
	//Done in ConfigChannels()
}

void VanetSim::ConfigMobility()
{
/*	Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
	ns2.Install (m_nodes.Begin(),m_nodes.End()-3);
	// configure movements for Car node, while reading trace file
	Ptr<MobilityModel> Temp = m_nodes.Get(nodeNum);//Controller
	Temp->SetPosition(Vector(0.0, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum+1);//source
	Temp->SetPosition(Vector(5.1, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum+2);//Sink
*/
	VMo->Install();
	double rt = VMo->GetReadTotalTime();
	if (duration<0)
	{
		duration = rt;
	}
	Time temp_now = Simulator::Now();
	std::cout<<"Now?"<<temp_now.GetSeconds ()<<std::endl;
	Ptr<MobilityModel> Temp = m_nodes.Get(nodeNum)->GetObject<MobilityModel>();//LC_1
	Temp->SetPosition(Vector(500.0, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum+1)->GetObject<MobilityModel>();//LC_2
	Temp->SetPosition(Vector(1500.0, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum+2)->GetObject<MobilityModel>();//LC_3
	Temp->SetPosition(Vector(2500.0, 0.0, 0.0));

    Temp = m_nodes.Get(nodeNum+3)->GetObject<MobilityModel>();//LC_4
	Temp->SetPosition(Vector(500.0, 1000.0, 0.0));
	Temp = m_nodes.Get(nodeNum+4)->GetObject<MobilityModel>();//LC_5
	Temp->SetPosition(Vector(1500.0, 1000.0, 0.0));
	Temp = m_nodes.Get(nodeNum+5)->GetObject<MobilityModel>();//LC_6
	Temp->SetPosition(Vector(2500.0, 1000.0, 0.0));

	Temp = m_nodes.Get(nodeNum+6)->GetObject<MobilityModel>();//LC_7
	Temp->SetPosition(Vector(500.0, 2000.0, 0.0));
	Temp = m_nodes.Get(nodeNum+7)->GetObject<MobilityModel>();//LC_8
	Temp->SetPosition(Vector(1500.0, 2000.0, 0.0));
	Temp = m_nodes.Get(nodeNum+8)->GetObject<MobilityModel>();//LC_9
	Temp->SetPosition(Vector(2500.0, 2000.0, 0.0));

	Temp = m_nodes.Get(nodeNum+9)->GetObject<MobilityModel>();//LC_10
	Temp->SetPosition(Vector(500.0, 3000.0, 0.0));
	Temp = m_nodes.Get(nodeNum+10)->GetObject<MobilityModel>();//LC_11
	Temp->SetPosition(Vector(1500.0, 3000.0, 0.0));
	Temp = m_nodes.Get(nodeNum+11)->GetObject<MobilityModel>();//LC_12
	Temp->SetPosition(Vector(2500.0, 3000.0, 0.0));

	Temp = m_nodes.Get(nodeNum+12)->GetObject<MobilityModel>();//LC_13
	Temp->SetPosition(Vector(0.0, 500.0, 0.0));
	Temp = m_nodes.Get(nodeNum+13)->GetObject<MobilityModel>();//LC_14
	Temp->SetPosition(Vector(0.0, 1500.0, 0.0));
	Temp = m_nodes.Get(nodeNum+14)->GetObject<MobilityModel>();//LC_15
	Temp->SetPosition(Vector(0.0, 2500.0, 0.0));

	Temp = m_nodes.Get(nodeNum+15)->GetObject<MobilityModel>();//LC_16
	Temp->SetPosition(Vector(1000.0, 500.0, 0.0));
	Temp = m_nodes.Get(nodeNum+16)->GetObject<MobilityModel>();//LC_17
	Temp->SetPosition(Vector(1000.0, 1500.0, 0.0));
	Temp = m_nodes.Get(nodeNum+17)->GetObject<MobilityModel>();//LC_18
	Temp->SetPosition(Vector(1000.0, 2500.0, 0.0));

	Temp = m_nodes.Get(nodeNum+18)->GetObject<MobilityModel>();//LC_19
	Temp->SetPosition(Vector(2000.0, 500.0, 0.0));
	Temp = m_nodes.Get(nodeNum+19)->GetObject<MobilityModel>();//LC_20
	Temp->SetPosition(Vector(2000.0, 1500.0, 0.0));
	Temp = m_nodes.Get(nodeNum+20)->GetObject<MobilityModel>();//LC_21
	Temp->SetPosition(Vector(2000.0, 2500.0, 0.0));

	Temp = m_nodes.Get(nodeNum+21)->GetObject<MobilityModel>();//LC_22
	Temp->SetPosition(Vector(3000.0, 500.0, 0.0));
	Temp = m_nodes.Get(nodeNum+22)->GetObject<MobilityModel>();//LC_23
	Temp->SetPosition(Vector(3000.0, 1500.0, 0.0));
	Temp = m_nodes.Get(nodeNum+23)->GetObject<MobilityModel>();//LC_24
	Temp->SetPosition(Vector(3000.0, 2500.0, 0.0));

	Temp = m_nodes.Get(nodeNum+24)->GetObject<MobilityModel>();//source
	Temp->SetPosition(Vector(1000.0, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum+25)->GetObject<MobilityModel>();//sink
	Temp->SetPosition(Vector(2000.0, 3000.0, 0.0));

	Temp = m_nodes.Get(nodeNum+26)->GetObject<MobilityModel>();//source2
	Temp->SetPosition(Vector(2000.0, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum+27)->GetObject<MobilityModel>();//sink2
	Temp->SetPosition(Vector(3000.0, 2000.0, 0.0));

	Temp = m_nodes.Get(nodeNum+28)->GetObject<MobilityModel>();//GC
	Temp->SetPosition(Vector(1500.0, 1500.0, 0.0));
}

void VanetSim::ConfigApp()
{
	//===Routing
	InternetStackHelper internet;
	if (mod == 0)
	{
		OlsrHelper olsr;
		//Ipv4ListRoutingHelper list;
		//list.Add(olsr,100);
		internet.SetRoutingHelper(olsr);
		std::cout<<"OLSR"<<std::endl;
		internet.Install (m_nodes);
	}
        else if (mod == 2)
	{
		AodvHelper aodv;
		internet.SetRoutingHelper(aodv);
		std::cout<<"AODV"<<std::endl;
		internet.Install (m_nodes);
	}
        else if (mod == 3)
	{
		DsrHelper dsr;
		//internet.SetRoutingHelper(dsr);
		DsrMainHelper dsrMain;
		std::cout<<"DSR"<<std::endl;
		internet.Install (m_nodes);
        	dsrMain.Install (dsr, m_nodes);
	}
        else if (mod == 4)
	{
		DsdvHelper dsdv;
		internet.SetRoutingHelper(dsdv);
		std::cout<<"DSDV"<<std::endl;
		internet.Install (m_nodes);
	}
	else
	{
	  SdnHelper sdn;
	  for (uint32_t i = 0; i<nodeNum; ++i)
	  {
	      sdn.SetNodeTypeMap (m_nodes.Get(i), sdn::CAR);
	  }

	  for(uint32_t i = nodeNum; i < nodeNum+24; i++)
	  {
	  	sdn.SetNodeTypeMap (m_nodes.Get (i), sdn::LOCAL_CONTROLLER);
	  }
	  sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+24), sdn::OTHERS);//Treat Source and Sink as CAR
	  sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+25), sdn::OTHERS);
	  sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+26), sdn::OTHERS);//Treat Source and Sink as CAR
	  sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+27), sdn::OTHERS);
	  sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+28), sdn::GLOBAL_CONTROLLER);
	  sdn.SetRLnSR (range1, range2);
	  internet.SetRoutingHelper(sdn);
		std::cout<<"SetRoutingHelper Done"<<std::endl;
	  internet.Install (m_nodes);
	}
	

	std::cout<<"internet.Install Done"<<std::endl;
	//===IP ADDRESS

	Ipv4AddressHelper ipv4S;
	NS_LOG_INFO ("Assign IP Addresses.");
	ipv4S.SetBase ("10.1.0.0", "255.255.0.0");//SCH
	m_SCHInterfaces = ipv4S.Assign (m_SCHDevices);
	std::cout<<"IPV4S Assigned"<<std::endl;

	Ipv4AddressHelper ipv4C;
	if (mod ==1)
	{
		NS_LOG_INFO ("Assign IP-C Addresses.");
		ipv4C.SetBase("192.168.0.0","255.255.0.0");//CCH
		m_CCHInterfaces = ipv4C.Assign(m_CCHDevices);
		std::cout<<"IPV4C Assigned"<<std::endl;
		for (uint32_t i = 0;i<m_nodes.GetN ();++i)
		  {
		    //std::cout<<"m_nodes.GetN () "<<i<<std::endl;
		    Ptr<sdn::RoutingProtocol> routing =
		        m_nodes.Get (i)->GetObject<sdn::RoutingProtocol> ();
        routing->SetCCHInterface (m_CCHInterfaces.Get (i).second);
		    routing->SetSCHInterface (m_SCHInterfaces.Get (i).second);
		  }
	}


	//===Traffic
	//source

	//onoff
	/*std::pair<Ptr<Ipv4>, uint32_t> RetValue = m_SCHInterfaces.Get (nodeNum+1);
	Ipv4InterfaceAddress theinterface = RetValue.first->GetAddress (RetValue.second, 0);
  Ipv4Address bcast = theinterface.GetLocal ().GetSubnetDirectedBroadcast (theinterface.GetMask ());*/
	Address remote (InetSocketAddress(m_SCHInterfaces.GetAddress(nodeNum+25), m_port));
	OnOffHelper Source("ns3::UdpSocketFactory",remote);//SendToSink
	Source.SetConstantRate(DataRate("8192bps"), 8192);
	Source.SetAttribute("OffTime",StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));


	m_source = Source.Install(m_nodes.Get(nodeNum+24));//Install on Source
	m_source.Start(Seconds(0));
	m_source.Stop(Seconds(duration));//Default Start time is 0.
	std::string temp = "/NodeList/"+std::to_string (nodeNum+24)+"/ApplicationList/0/$ns3::OnOffApplication/Tx";

	Config::ConnectWithoutContext (
	    temp,
	    MakeCallback(&VanetSim::TXTrace, this));
	/*
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	source = Socket::CreateSocket (m_nodes.Get(nodeNum+1), tid);
	Simulator::Schedule(Seconds(0.0), &VanetSim::SendDataPacket, this);
	*/

	//sink
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Ptr<Socket> sink = Socket::CreateSocket (m_nodes.Get(nodeNum+25), tid);//The Sink
  //HearALL;
	//InetSocketAddress local = InetSocketAddress(m_CCHInterfaces.GetAddress(nodeNum+2),m_port);
	InetSocketAddress local = InetSocketAddress(Ipv4Address::GetZero (),m_port);
	sink->Bind(local);
	sink->SetRecvCallback(MakeCallback(&VanetSim::ReceiveDataPacket, this));







	Address remote2 (InetSocketAddress(m_SCHInterfaces.GetAddress(nodeNum+27), m_port2));
	OnOffHelper Source2("ns3::UdpSocketFactory",remote2);//SendToSink
	Source2.SetConstantRate(DataRate("10kbps"));
	Source2.SetAttribute("OffTime",StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));


	m_source2 = Source2.Install(m_nodes.Get(nodeNum+26));//Install on Source
	m_source2.Start(Seconds(0));
	m_source2.Stop(Seconds(duration));//Default Start time is 0.
	std::string temp2 = "/NodeList/"+std::to_string (nodeNum+26)+"/ApplicationList/0/$ns3::OnOffApplication/Tx";

	Config::ConnectWithoutContext (

	    temp2,
	    MakeCallback(&VanetSim::TXTrace2, this));
	/*
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	source = Socket::CreateSocket (m_nodes.Get(nodeNum+1), tid);
	Simulator::Schedule(Seconds(0.0), &VanetSim::SendDataPacket, this);
	*/

	//sink
	TypeId tid2 = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Ptr<Socket> sink2 = Socket::CreateSocket (m_nodes.Get(nodeNum+27), tid2);//The Sink
  //HearALL;
	//InetSocketAddress local = InetSocketAddress(m_CCHInterfaces.GetAddress(nodeNum+2),m_port);
	InetSocketAddress local2 = InetSocketAddress(Ipv4Address::GetZero (),m_port2);
	sink2->Bind(local2);
	sink2->SetRecvCallback(MakeCallback(&VanetSim::ReceiveDataPacket2, this));
}

void VanetSim::ReceiveDataPacket(Ptr<Socket> socket)
{
	Ptr<Packet> packet;
	while ((packet = socket->Recv()))
	{

		// sdn::MessageHeader messageHeader;
  //     	if (packet->RemoveHeader (messageHeader) == 0)
  //     		NS_ASSERT (false);
  //     	TTL += 64 - int (messageHeader.GetTimeToLive ());
		Rx_Data_Bytes += packet->GetSize();
		Rx_Data_Pkts++;
		std::cout<<".";
	        uint64_t uid = packet->GetUid ();
	        if (dup_det.find (uid) == dup_det.end ())
	        {
	        	//Unique_RX_Pkts++;
	       		 dup_det.insert (uid);

	       		 Time now = Simulator::Now ();
	        	 int64_t temp = now.GetMicroSeconds () - delay[uid].GetMicroSeconds ();
	       		 delay_vector.push_back (temp);
	        }
	}   
}


void VanetSim::ReceiveDataPacket2(Ptr<Socket> socket)
{
	Ptr<Packet> packet;
	while ((packet = socket->Recv()))
	{

		// sdn::MessageHeader messageHeader;
  //     	if (packet->RemoveHeader (messageHeader) == 0)
  //     		NS_ASSERT (false);
  //     	TTL += 64 - int (messageHeader.GetTimeToLive ());
		Rx_Data_Bytes2 += packet->GetSize();
		Rx_Data_Pkts2++;
		std::cout<<".";
	        uint64_t uid = packet->GetUid ();
	        if (dup_det2.find (uid) == dup_det2.end ())
	        {
	        	//Unique_RX_Pkts++;
	       		 dup_det2.insert (uid);

	       		 Time now = Simulator::Now ();
	        	 int64_t temp = now.GetMicroSeconds () - delay2[uid].GetMicroSeconds ();
	       		 delay_vector2.push_back (temp);
	        }
	}   
}

void VanetSim::SendDataPacket()
{
	/*Ptr<Packet> packet = Create<Packet> (packetSize);
	source->SendTo(packet, 0, )
	Simulator::Schedule(Seconds(interval), &VanetSim::SendDataPacket, this);*/
	//TODO
}

void VanetSim::ConfigTracing()
{
	//TODO
}

void VanetSim::ProcessOutputs()
{
	std::cout<<Tx_Data_Pkts<<std::endl;
	std::cout<<Rx_Data_Pkts<<std::endl;

	os<<"Result:"<<std::endl;
  	os<<"Tx_Data_Pkts:   "<<Tx_Data_Pkts<<std::endl;
        os<<"Rx_Data_Pkts3:   "<<Rx_Data_Pkts<<std::endl;

        if (!delay_vector.empty ())
	{
              int64_t best = delay_vector[0],
              worst = delay_vector[0];
              double avg = 0;
              for (std::vector<int64_t>::const_iterator cit = delay_vector.begin ();
              cit != delay_vector.end ();++cit)
              {
                if (*cit<best)
                {
                  best = *cit;
                }

                if (*cit>worst)
                {
                  worst = *cit;
                }
                avg += *cit;
              }

              avg /= delay_vector.size();
              std::cout<<"Best delay:   "<<best<<"us"<<std::endl;
              std::cout<<"Worst delay:   "<<worst<<"us"<<std::endl;
              std::cout<<"Avg delay: "<<avg<<"us"<<std::endl;
             // std::cout << "avg TTL" << TTL/delay_vector.size() << std::endl;
   	      	  os<<"Best delay:   "<<best<<"us"<<std::endl;
              os<<"Worst delay:   "<<worst<<"us"<<std::endl;
              os<<"Avg delay: "<<avg<<"us"<<std::endl;
              os << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
	}
	else
	{
	    std::cout<<"NO PACKETS WERE RECEIVED."<<std::endl;
	    os<<"NO PACKETS WERE RECEIVED."<<std::endl;
	}

	std::cout<<Tx_Data_Pkts2<<std::endl;
	std::cout<<Rx_Data_Pkts2<<std::endl;
	
	os<<"Result:"<<std::endl;
  	os<<"Tx_Data_Pkts:   "<<Tx_Data_Pkts2<<std::endl;
        os<<"Rx_Data_Pkts3:   "<<Rx_Data_Pkts2<<std::endl;

        if (!delay_vector2.empty ())
	{
              int64_t best = delay_vector2[0],
              worst = delay_vector2[0];
              double avg = 0;
              for (std::vector<int64_t>::const_iterator cit = delay_vector2.begin ();
              cit != delay_vector2.end ();++cit)
              {
                if (*cit<best)
                {
                  best = *cit;
                }

                if (*cit>worst)
                {
                  worst = *cit;
                }
                avg += *cit;
              }

              avg /= delay_vector2.size();
              std::cout<<"Best delay:   "<<best<<"us"<<std::endl;
              std::cout<<"Worst delay:   "<<worst<<"us"<<std::endl;
              std::cout<<"Avg delay: "<<avg<<"us"<<std::endl;
             // std::cout << "avg TTL" << TTL/delay_vector.size() << std::endl;
   	      	  os<<"Best delay:   "<<best<<"us"<<std::endl;
              os<<"Worst delay:   "<<worst<<"us"<<std::endl;
              os<<"Avg delay: "<<avg<<"us"<<std::endl;
              os << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
	}
	else
	{
	    std::cout<<"NO PACKETS WERE RECEIVED."<<std::endl;
	    os<<"NO PACKETS WERE RECEIVED."<<std::endl;
	}

}


// void VanetSim::ProcessOutputs2()
// {

// 	std::cout<<Tx_Data_Pkts2<<std::endl;
// 	std::cout<<Rx_Data_Pkts2<<std::endl;

// 	os<<"Result:"<<std::endl;
//   	os<<"Tx_Data_Pkts:   "<<Tx_Data_Pkts2<<std::endl;
//         os<<"Rx_Data_Pkts3:   "<<Rx_Data_Pkts2<<std::endl;

//     if (!delay_vector2.empty ())
// 	{
//               int64_t best = delay_vector2[0],
//               worst = delay_vector2[0];
//               double avg = 0;
//               for (std::vector<int64_t>::const_iterator cit = delay_vector2.begin ();
//               cit != delay_vector2.end ();++cit)
//               {
//                 if (*cit<best)
//                 {
//                   best = *cit;
//                 }

//                 if (*cit>worst)
//                 {
//                   worst = *cit;
//                 }
//                 avg += *cit;
//               }

//               avg /= delay_vector2.size();
//               std::cout<<"Best delay2:   "<<best<<"us"<<std::endl;
//               std::cout<<"Worst delay2:   "<<worst<<"us"<<std::endl;
//               std::cout<<"Avg delay2: "<<avg<<"us"<<std::endl;
//              // std::cout << "avg TTL" << TTL/delay_vector.size() << std::endl;
//    	      	  os<<"Best delay:   "<<best<<"us"<<std::endl;
//               os<<"Worst delay:   "<<worst<<"us"<<std::endl;
//               os<<"Avg delay: "<<avg<<"us"<<std::endl;
//               os << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
// 	}
// 	else
// 	{
// 	    std::cout<<"NO PACKETS WERE RECEIVED."<<std::endl;
// 	    os<<"NO PACKETS WERE RECEIVED."<<std::endl;
// 	}

// }

void VanetSim::Run()
{
	Simulator::Schedule(Seconds(0.0), &VanetSim::Look_at_clock, this);
	//Simulator::Schedule(Seconds(0.5), &VanetSim::Look_at_clock2, this);
	std::cout << "Starting simulation for " << duration << " s ..."<< std::endl;
	os << "Starting simulation for " << duration << " s ..."<< std::endl;
	Simulator::Stop(Seconds(duration));
	Simulator::Run();
	Simulator::Destroy();

}

void VanetSim::Look_at_clock()
{
	std::cout<<"Now:"<<Simulator::Now().GetSeconds()<<std::endl;
	os<<"Now:  "<<Simulator::Now().GetSeconds()
  	<<"Tx_Data_Pkts:   "<<Tx_Data_Pkts
  	<<"Rx_Data_Pkts:   "<<Rx_Data_Pkts<<std::endl;
	/*Ptr<MobilityModel> Temp = m_nodes.Get (nodeNum)->GetObject<MobilityModel>();
  std::cout<<Temp->GetPosition().x<<","<<Temp->GetPosition().y<<","<<Temp->GetPosition().z<<std::endl;
  std::cout<<Temp->GetVelocity().x<<","<<Temp->GetVelocity().y<<","<<Temp->GetVelocity().z<<std::endl;
  Temp = m_nodes.Get (nodeNum+1)->GetObject<MobilityModel>();
  std::cout<<Temp->GetPosition().x<<","<<Temp->GetPosition().y<<","<<Temp->GetPosition().z<<std::endl;
  std::cout<<Temp->GetVelocity().x<<","<<Temp->GetVelocity().y<<","<<Temp->GetVelocity().z<<std::endl;
  Temp = m_nodes.Get (nodeNum+2)->GetObject<MobilityModel>();
  std::cout<<Temp->GetPosition().x<<","<<Temp->GetPosition().y<<","<<Temp->GetPosition().z<<std::endl;
  std::cout<<Temp->GetVelocity().x<<","<<Temp->GetVelocity().y<<","<<Temp->GetVelocity().z<<std::endl;
  */
	/*
	os<<"Now:"<<Simulator::Now().GetSeconds()<<std::endl;
	Ptr<OutputStreamWrapper> osw = Create<OutputStreamWrapper> (&std::cout);
	m_nodes.Get(nodeNum+1)->GetObject<Ipv4>()->GetRoutingProtocol()->PrintRoutingTable(osw);
	Ptr<OutputStreamWrapper> osw2 = Create<OutputStreamWrapper> (&os);
	m_nodes.Get(nodeNum+1)->GetObject<Ipv4>()->GetRoutingProtocol()->PrintRoutingTable(osw2);
	*/
	/*2  Ptr<MobilityModel> Temp;
	Vector vt;
	for (int i = 0;i<=nodeNum+2;++i)
	{
		Temp = m_nodes.Get(i)->GetObject<MobilityModel>();
		vt = Temp->GetPosition();
		std::cout<<i<<":"<<vt.x<<","<<vt.y<<","<<vt.z<<";"<<std::flush;
	}
	std::cout<<std::endl;*/
	ProcessOutputs();
	Simulator::Schedule(Seconds(1.0), &VanetSim::Look_at_clock, this);
}

/*
void VanetSim::Look_at_clock2()
{
	std::cout<<"Now:"<<Simulator::Now().GetSeconds()<<std::endl;
	std::cout << "!!!" << std::endl;
	os<<"Now:  "<<Simulator::Now().GetSeconds()
  	<<"Tx_Data_Pkts:   "<<Tx_Data_Pkts2
  	<<"Rx_Data_Pkts:   "<<Rx_Data_Pkts2<<std::endl;
	/*Ptr<MobilityModel> Temp = m_nodes.Get (nodeNum)->GetObject<MobilityModel>();
  std::cout<<Temp->GetPosition().x<<","<<Temp->GetPosition().y<<","<<Temp->GetPosition().z<<std::endl;
  std::cout<<Temp->GetVelocity().x<<","<<Temp->GetVelocity().y<<","<<Temp->GetVelocity().z<<std::endl;
  Temp = m_nodes.Get (nodeNum+1)->GetObject<MobilityModel>();
  std::cout<<Temp->GetPosition().x<<","<<Temp->GetPosition().y<<","<<Temp->GetPosition().z<<std::endl;
  std::cout<<Temp->GetVelocity().x<<","<<Temp->GetVelocity().y<<","<<Temp->GetVelocity().z<<std::endl;
  Temp = m_nodes.Get (nodeNum+2)->GetObject<MobilityModel>();
  std::cout<<Temp->GetPosition().x<<","<<Temp->GetPosition().y<<","<<Temp->GetPosition().z<<std::endl;
  std::cout<<Temp->GetVelocity().x<<","<<Temp->GetVelocity().y<<","<<Temp->GetVelocity().z<<std::endl;
  */
	/*
	os<<"Now:"<<Simulator::Now().GetSeconds()<<std::endl;
	Ptr<OutputStreamWrapper> osw = Create<OutputStreamWrapper> (&std::cout);
	m_nodes.Get(nodeNum+1)->GetObject<Ipv4>()->GetRoutingProtocol()->PrintRoutingTable(osw);
	Ptr<OutputStreamWrapper> osw2 = Create<OutputStreamWrapper> (&os);
	m_nodes.Get(nodeNum+1)->GetObject<Ipv4>()->GetRoutingProtocol()->PrintRoutingTable(osw2);
	*/
	/*2  Ptr<MobilityModel> Temp;
	Vector vt;
	for (int i = 0;i<=nodeNum+2;++i)
	{
		Temp = m_nodes.Get(i)->GetObject<MobilityModel>();
		vt = Temp->GetPosition();
		std::cout<<i<<":"<<vt.x<<","<<vt.y<<","<<vt.z<<";"<<std::flush;
	}
	std::cout<<std::endl;
	ProcessOutputs2();
	Simulator::Schedule(Seconds(1.0), &VanetSim::Look_at_clock2, this);
}*/

void
VanetSim::TXTrace (Ptr<const Packet> newpacket)
{
	//std::cout << "TXTrace" << std::endl;
  Tx_Data_Pkts++;
  Tx_Data_Bytes += newpacket->GetSize ();
  Time now = Simulator::Now ();
  delay[newpacket->GetUid ()] = now;
  //std::cout<<"ANOTHER ONE!HAHAHA"<<std::endl;
}

void
VanetSim::TXTrace2 (Ptr<const Packet> newpacket)
{
  Tx_Data_Pkts2++;
  Tx_Data_Bytes2 += newpacket->GetSize ();
  Time now = Simulator::Now ();
  delay2[newpacket->GetUid ()] = now;
  //std::cout << "TXTrace2" << std::endl;
  //std::cout<<"ANOTHER ONE!HAHAHA"<<std::endl;
}

// Example to use ns2 traces file in ns3
int main (int argc, char *argv[])
{
	VanetSim SDN_test;
	//std::cout << "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||" << std::endl;
	SDN_test.Simulate(argc, argv);
	return 0;
}




