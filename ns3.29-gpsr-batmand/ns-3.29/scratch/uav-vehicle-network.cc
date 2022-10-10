#include "ns3/netanim-module.h"
#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "uav-vehicle-network.h"
#include "ns3/aodv-helper.h"
#include "ns3/dsdv-helper.h"
#include "ns3/olsr-helper.h"
#include "ns3/gpsr-helper.h"
#include "ns3/batmand-helper.h"
#include "ns3/buildings-module.h"
#include "ns3/buildings-helper.h"
#include "ns3/mobility-building-info.h"
#include "ns3/buildings-propagation-loss-model.h"
#include "ns3/building.h"
#include "ns3/spectrum-helper.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/spectrum-analyzer-helper.h"
#include "ns3/spectrum-channel.h"
#include "ns3/oh-buildings-propagation-loss-model.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/circle-mobility.h"
//#include "SpiralModel.h"

#include <iostream>
#include <map>
#include <sstream>
#include <string.h>
#include<fstream>


using namespace ns3;
using namespace std;


NS_LOG_COMPONENT_DEFINE ("UavVehicleNetwork");

UavVehicleNetwork::UavVehicleNetwork ()
{
}

UavVehicleNetwork::~UavVehicleNetwork ()
{
}

double rad(int a,int b,int r){
  double R ;
  srand(r);//设置随机数种子，使每次产生的随机序列不同
		 R = rand() % (b*100) / double(100);
		R+=a;
  return R;
}

string a1,a2,a3,a4;
string b1,b2,b3,b4;
string c1,c2,c3,c4;
string d1,d2,d3,d4;
string e1,e2,e3,e4;

std::stringstream s1;
std::stringstream s2;
std::stringstream s3;
std::stringstream s4;

Ipv4StaticRoutingHelper staticRouting;

// 1.设定参数
void
UavVehicleNetwork::SetArgument (const std::map<std::string, std::string> &arguments)
{
  for (auto it : arguments)
    {
      if (it.first == "nWifi")
        this->nWifi = stoi (it.second);
      else if (it.first == "appNum")
        this->appNum = stoi (it.second);
      else if (it.first == "maxPackets")
        this->maxPackets = stoi (it.second);
      else if (it.first == "interval")
        this->interval = stod (it.second);
      else if (it.first == "packetSize")
        this->packetSize = stoi (it.second);
      else if (it.first == "appServerPort")
        this->appServerPort = stoi (it.second);
      else if (it.first == "appServerBeginTime")
        this->appServerBeginTime = stoi (it.second);
      else if (it.first == "appServerEndTime")
        this->appServerEndTime = stoi (it.second);
      else if (it.first == "appClientBeginTime")
        this->appClientBeginTime = stoi (it.second);
      else if (it.first == "appClientEndTime")
        this->appClientEndTime = stoi (it.second);
      else if (it.first == "simulatorEndTime")
        this->simulatorEndTime = stoi (it.second);
      else
        NS_LOG_UNCOND ("No parameters" + it.first);
    }
}

/**
   * 2（可选）.设定路由协议
   * 注意：如果调用的话，就需要在初始化环境的时候SetRoutingHelper
   */
void
UavVehicleNetwork::SetRoutingHelper (const Ipv4RoutingHelper &routing,int priority)
{
  //this->routing = routing.Copy ();
  list.Add(staticRouting,0);
  list.Add(routing,priority);
}

/**
   * 3（可选）.设定模拟器
   * 注意：如果没有调用的话，就直接使用默认的模拟器Simulator
   */
void
UavVehicleNetwork::SetSimulator (Ptr<SimulatorImpl> simulator)
{
  Simulator::SetImplementation (simulator);
}

// 4.初始化环境
void
UavVehicleNetwork::SetupEnvironment ()
{
  // 日志打印
  // LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
  // LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);

  // 创建网络节点
  NodeContainer wifiAdhocNodes;
  wifiAdhocNodes.Create (this->nWifi);

  // 设置wifi物理层连接通道
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  
  //wifiChannel.AddPropagationLoss ("ns3::OhBuildingsPropagationLossModel");


  // wifiChannel.AddPropagationLoss ("ns3::OhBuildingsPropagationLossModel","Frequency", DoubleValue (15.150e9));
  // Ptr<OhBuildingsPropagationLossModel> ss = wifiChannel->GetObject<OhBuildingsPropagationLossModel> ();  
  //             ss->SetAttribute("Frequency", 
  //                  DoubleValue (15.150e9));
  // wifiChannel.AddPropagationLoss ("ns3::HybridBuildingsPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());

  // 设置数据链路层 wifi节点类型
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");

  // 为节点安装网络设备 并配置节点的物理层、数据链路层
  WifiHelper wifi;
  NetDeviceContainer wifiAdhocDevices = wifi.Install (wifiPhy, wifiMac, wifiAdhocNodes);

  // 设置网络协议栈 配置节点网络层
  InternetStackHelper stack;
  // if (this->routing != NULL)
  //   {
  //     stack.SetRoutingHelper (*this->routing);
  //   }
  stack.SetRoutingHelper(list);
  stack.Install (wifiAdhocNodes);

  // 分配IP地址
  Ipv4AddressHelper address;
  address.SetBase (Ipv4Address (this->baseIp.c_str ()), Ipv4Mask (this->subNetMask.c_str ()));
  Ipv4InterfaceContainer AdhocInterfaces;
  AdhocInterfaces = address.Assign (wifiAdhocDevices);

  // 设置移动模型
  MobilityHelper mobility;
  mobility.SetPositionAllocator(
        "ns3::RandomBoxPositionAllocator", "X",
        StringValue("ns3::UniformRandomVariable[Min=0|Max=50]"), "Y",
        StringValue("ns3::UniformRandomVariable[Min=0|Max=50]"), "Z",
        StringValue("ns3::UniformRandomVariable[Min=0|Max=5]"));
  speed = 5.0;
  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel", "Mode", StringValue ("Distance"), "Distance",
                             StringValue ("5"), "Speed",
                             StringValue ("ns3::ConstantRandomVariable[Constant=" + std::to_string(speed) +"]"), "Bounds",
                             StringValue ("0|50|0|50"));

  //uint32_t num = this->nWifi / 3;

  // 无人机随机运动
  for (size_t i = 0; i < nWifi/2; i++)
    {
      mobility.Install (wifiAdhocNodes.Get (i));
    }

  mobility.SetMobilityModel ("ns3::GaussMarkovMobilityModel", "Bounds",
                             BoxValue (Box (0, 10, 0, 10, 0, 10)), "TimeStep",
                             TimeValue (Seconds (0.5)), "Alpha", DoubleValue (0.85));

  // 车辆匀速运动
mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
   
  for (size_t i = nWifi/2; i < nWifi; i++)
    {
      mobility.Install (wifiAdhocNodes.Get (i));
      Ptr<ConstantVelocityMobilityModel> mod = wifiAdhocNodes.Get (i)->GetObject<ConstantVelocityMobilityModel> ();
      double x=rad(0,10,i);
      double y=rad(0,10,i+10);
      double xs=rad(0,1,i);
      double ys=rad(0,1,i+110);
      mod->SetPosition (Vector (x,y, 1.0));
      mod->SetVelocity (Vector (xs,ys, 0));
    }

  // // 将1/3节点设置成螺旋运动
  // mobility.SetMobilityModel ("ns3::SpiralMobilityModel");
  // for (size_t i = nWifi - num; i < nWifi; i++)
  //   {
  //     mobility.Install (wifiAdhocNodes.Get (i));
  //     Ptr<SpiralMobilityModel> mod = wifiAdhocNodes.Get (i)->GetObject<SpiralMobilityModel> ();
  //     double x=rad(0,10,i+100);
  //     double y=rad(0,10,i+110);
  //  // std::cout<<x<<" "<<y<<endl; 
  //     mod->SetParameters (Vector (x,y, 1.0), 1.0, 2.0, 2.0);
  //     mod->SetZSpeed (0.1);
  //   }

  BuildingsHelper::Install (wifiAdhocNodes);
  

  // 设置应用层 服务器 appNum为指定的客户端和服务器的总数量
  ApplicationContainer serverApps;
  uint8_t i = 0;
  uint32_t tmpWifi = this->nWifi;
  int tmpNum = this->appNum / 2;
  // std::cout << "服务器数量" << tmpNum << endl;
  
  while (tmpNum--)
    {
      // std::cout << this->appServerPort + i << endl;
      UdpEchoServerHelper echoServer (this->appServerPort + i++);
      serverApps.Add (echoServer.Install (wifiAdhocNodes.Get (--tmpWifi)));
       flowNodes.Add(wifiAdhocNodes.Get (tmpWifi));
       server.push_back(tmpWifi);
      
    }
    
  // 设置应用层 客户端
  ApplicationContainer clientApps;
  uint32_t tmpWifi2 = this->nWifi;
  int tmpNum2 = this->appNum / 2;
  uint8_t j = 0, k = 0;
  while (tmpNum2--)
    {
      // std::cout << this->appServerPort + j << endl;
      UdpEchoClientHelper echoClient (AdhocInterfaces.GetAddress (--tmpWifi2),
                                      this->appServerPort + j++);
                                   
      echoClient.SetAttribute ("MaxPackets", UintegerValue (this->maxPackets));
      echoClient.SetAttribute ("Interval", TimeValue (Seconds (this->interval)));
      echoClient.SetAttribute ("PacketSize", UintegerValue (this->packetSize));
      clientApps.Add (echoClient.Install (wifiAdhocNodes.Get (k++)));
         flowNodes.Add(wifiAdhocNodes.Get (k-1));
             client.push_back(k-1);
    }
   // monitor = flowmon.Install(flowNodes);
  monitor = flowmon.Install(wifiAdhocNodes);
  serverApps.Start (Seconds (this->appServerBeginTime));
  serverApps.Stop (Seconds (this->appServerEndTime));
  clientApps.Start (Seconds (this->appClientBeginTime));
  clientApps.Stop (Seconds (this->appClientEndTime));

  // AsciiTraceHelper ascii;
  // wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("uav-vehicle-network.tr"));
}

// 5.运行
void
UavVehicleNetwork::Run ()
{
  SetupEnvironment ();

  // // 生成可视化文件
  //AnimationInterface anim ("uav-vehicle-network.xml");
  // anim.UpdateNodeDescription (this->nWifi - 1, "server");
  // anim.UpdateNodeDescription (0, "client");

  Simulator::Stop (Seconds (this->simulatorEndTime));
  Simulator::Run ();

  Simulator::Destroy ();
}

// 6.获取仿真结果
std::map<std::string, std::string> *
UavVehicleNetwork::GetResult ()
{
  // 时延 丢包率 吞吐量 抖动
  uint32_t SentPackets = 0;
  uint32_t ReceivedPackets = 0;
  uint32_t LostPackets = 0;

  // 统计结果
  int j = 0;
  float AvgThroughput = 0;
  Time Jitter;
  Time Delay;

  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin ();
       iter != stats.end (); ++iter)
    {
      //Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first);
      // NS_LOG_UNCOND ("----Flow ID:" << iter->first);
      // NS_LOG_UNCOND ("Src Addr" << t.sourceAddress << " ------> Dst Addr " << t.destinationAddress);
      // NS_LOG_UNCOND ("Sent Packets=" << iter->second.txPackets);
      // NS_LOG_UNCOND ("Received Packets =" << iter->second.rxPackets);
      // NS_LOG_UNCOND ("Lost Packets =" << iter->second.txPackets - iter->second.rxPackets);
      // NS_LOG_UNCOND ("Packet delivery ratio ="
      //                << iter->second.rxPackets * 100 / iter->second.txPackets << "%");
      // NS_LOG_UNCOND ("Packet loss ratio =" << (iter->second.txPackets - iter->second.rxPackets) *
      //                                             100 / iter->second.txPackets
      //                                      << "%");
      // NS_LOG_UNCOND ("Delay =" << iter->second.delaySum);
      // NS_LOG_UNCOND ("Jitter =" << iter->second.jitterSum);
      // NS_LOG_UNCOND ("Throughput =" << iter->second.rxBytes * 8.0 /
      //                                      (iter->second.timeLastRxPacket.GetSeconds () -
      //                                       iter->second.timeFirstTxPacket.GetSeconds ()) /
      //                                      1024
      //                               << "Kbps");

      SentPackets = SentPackets + (iter->second.txPackets);
      ReceivedPackets = ReceivedPackets + (iter->second.rxPackets);
      LostPackets = LostPackets + (iter->second.txPackets - iter->second.rxPackets);
      AvgThroughput = AvgThroughput + (iter->second.rxBytes * 8.0 /
                                       (iter->second.timeLastRxPacket.GetSeconds () -
                                        iter->second.timeFirstTxPacket.GetSeconds ()) /
                                       1024);
      Delay = Delay + (iter->second.delaySum);
      Jitter = Jitter + (iter->second.jitterSum);

      j = j + 1;
    }

  AvgThroughput = AvgThroughput / j;
  // NS_LOG_UNCOND ("--------Total Results of the simulation----------" << std::endl);
  // NS_LOG_UNCOND ("Total sent packets  =" << SentPackets);
  // NS_LOG_UNCOND ("Total Received Packets =" << ReceivedPackets);
  // NS_LOG_UNCOND ("Total Lost Packets =" << LostPackets);
  // NS_LOG_UNCOND ("Packet Loss ratio =" << ((LostPackets * 100) / SentPackets) << "%");
  // NS_LOG_UNCOND ("Packet delivery ratio =" << ((ReceivedPackets * 100) / SentPackets) << "%");
  // NS_LOG_UNCOND ("Average Throughput =" << AvgThroughput << "Kbps");
  // NS_LOG_UNCOND ("End to End Delay =" << Delay);
  // NS_LOG_UNCOND ("End to End Jitter delay =" << Jitter);
  // NS_LOG_UNCOND ("Total Flod id " << j);
  // monitor->SerializeToXmlFile ("manet-routing.xml", true, true);

  s1.str("");
  s2.str("");
  s3.str("");
  s4.str("");


  if(SentPackets!=0)
  s1 << (LostPackets * 100) / SentPackets;
  else
  s1 << 10000;
  rs["Packet loss rate"] = s1.str () + "%";
  // std::cout << AvgThroughput << endl;
  s2 << AvgThroughput;
  rs["Average throughput"] = s2.str () + "Kbps";
  // std::cout << Delay << endl;
  s3 << Delay.GetMilliSeconds() / ReceivedPackets;
  rs["Average time delay"] = s3.str ();
  // std::cout << Jitter << endl;
  s4 << Jitter.GetMilliSeconds() / ReceivedPackets;
  rs["Jitter delay"] = s4.str ();
    
  std::map<std::string, std::string> *ans = &rs;
  for (std::map<std::string, std::string>::iterator iter = (*ans).begin (); iter != (*ans).end ();
       ++iter)
    {
      std::cout << iter->first << " " << iter->second << endl;
    }
  
  return ans;
}

int
main ()
{
  UavVehicleNetwork network1;
  UavVehicleNetwork network2;
  UavVehicleNetwork network3;
  UavVehicleNetwork network4;
  UavVehicleNetwork network5;
  std::map<std::string, std::string> params;

  params["nWifi"] = "30";
  params["appNum"] = "30";
  params["maxPackets"] = "10";
  params["appServerEndTime"] = "200.0";
  params["appClientEndTime"] = "200.0";
  params["simulatorEndTime"] = "200.0";
  params["appServerBeginTime"] = "1.0";
  params["appClientBeginTime"] = "2.0";
  params["interval"] = "1";

  network1.SetArgument (params);
  AodvHelper aodv;
  DsdvHelper dsdv;
  OlsrHelper olsr;
  GpsrHelper gpsr;
  BatmandHelper batman;
  network1.SetRoutingHelper (aodv,10);
  network1.Run ();
  network1.GetResult ();
  a1 = s1.str();
  a2 = s2.str();
  a3 = s3.str();
  a4 = s4.str();
  
 
  network2.SetArgument (params);
  network2.SetRoutingHelper (dsdv,10);
  network2.Run ();
  network2.GetResult ();
  b1 = s1.str();
  b2 = s2.str();
  b3 = s3.str();
  b4 = s4.str();

  network3.SetArgument (params);
  network3.SetRoutingHelper (olsr,12);
  network3.Run ();
  network3.GetResult ();
  c1 = s1.str();
  c2 = s2.str();
  c3 = s3.str();
  c4 = s4.str();
  
  network4.SetArgument (params);
  network4.SetRoutingHelper (gpsr,13);
  network4.Run ();
  network4.GetResult ();
  d1 = s1.str();
  d2 = s2.str();
  d3 = s3.str();
  d4 = s4.str();
  
  network5.SetArgument (params);
  network5.SetRoutingHelper (batman,14);
  network5.Run ();
  network5.GetResult ();
  e1 = s1.str();
  e2 = s2.str();
  e3 = s3.str();
  e4 = s4.str();

  ofstream oFile; 
	oFile.open("data.csv", ios::out | ios::trunc);    
	oFile <<"Speed == " << network1.speed << "," << "Packet loss rate(%)" << "," << "Average time delay(ms)" << "," << "Jitter delay(ms)" << "," << "Average throughput(kbps)" << endl; 
  oFile << "aodv" << "," << a1 << "," << a3 << "," << a4 << "," << a2 << endl;  
  oFile << "dsdv" << "," << b1 << "," << b3 << "," << b4 << "," << b2 << endl;
  oFile << "olsr" << "," << c1 << "," << c3 << "," << c4 << "," << c2 << endl;
  oFile << "gpsr" << "," << d1 << "," << d3 << "," << d4 << "," << d2 << endl;
  oFile << "batman" << "," << e1 << "," << e3 << "," << e4 << "," << e2 << endl;
	oFile.close();
  return 0;
}