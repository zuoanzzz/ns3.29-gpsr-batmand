#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ssid.h"
#include "ns3/yans-wifi-helper.h"

#include "ns3/aodv-helper.h"
#include "ns3/batmand-helper.h"
#include "ns3/dsdv-helper.h"
#include "ns3/dsr-helper.h"
#include "ns3/gpsr-helper.h"
#include "ns3/olsr-helper.h"

#include "ns3/dsr-main-helper.h"

#include "ns3/SpiralModel.h"
#include "ns3/circle-mobility.h"
#include "ns3/flow-monitor-module.h"

#include <unistd.h>
#include <iostream>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("UavVehicleNetwork");

class UavVehicleNetwork{
   private:
    Ptr<UniformRandomVariable> uv;
    NodeContainer nodes;
    NetDeviceContainer devices;
    std::string phyMode;
    Ipv4InterfaceContainer interface;
    Ipv4ListRoutingHelper list;
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    InternetStackHelper internet;

    double height;
    bool tracing;
    uint32_t packetSize; /* 每个包的大小 */
    uint32_t MaxPackets; /* 每个节点发送的包的数量 */
    double interval;     /* 发送每个包的时间间隔 */
    double stopTime;     /* 设置网络模拟停止时间 */
    uint32_t nodeNumber; /* 节点的总数量 */
    double distance;
    double startTime;     /* 开始模拟时间 */
    int sarNumber;        /* 发送和接受节点的数量 */
    double transDistance; /* 传输距离 */
    int seed;             /* 设置随机种子 */
    double mspeed; /* 设置节点速度：除了匀速运动和高斯运动 */

    Time interPacketInterval; /* 发送包的时间间隔 */

    int priority; /* 每个协议的优先级 */
    uint32_t serverPort;

    std::vector<int> client;
    int server;

    double clientStartTime;
    double serverStartTime;

    double clientStopTime;
    double serverStopTime;

    bool usingDsr;

    /* result需要的变量 */
    int sendNum;
    int recvNum;
    int lostNum;
    double lossRatio;
    double deliveryRatio;
    double ETEDelay;
    double jitterDelay;
    double avgThroughput;

    std::map<std::string, std::string> result;

    bool FlowCondition(Ipv4FlowClassifier::FiveTuple t);

   public:
    UavVehicleNetwork();
    ~UavVehicleNetwork();

    void CreateNodes();
    void CreateDevices();
    void InstallInternetStack();
    void InstallApplications();
    void InstallSocket(int server, int client);
    void SetUsingDsr(bool usinDsr);

    void SetArgument(const std::map<std::string, std::string>& arguments);
    void SetRoutingHelper(const Ipv4RoutingHelper& routing);
    void SetSimulator(Ptr<SimulatorImpl> simulator);
    void SetupEnvironment();
    void Run();
    std::map<std::string, std::string>* GetResult();

    void SetDsr();
    void resetResult();

    /* DSR协议使用静态变量获取网络数据 */
    static uint32_t recvPacketNumber;
    static uint32_t sendPacketNumber;
    static uint32_t recvByte;
    static int64_t txFirstPacketTime;
    static int64_t rxLastPacketTime;
    static std::vector<int64_t> delay;

    static void ReceivePacket(Ptr<Socket> socket);
    static void GenerateTraffic(Ptr<Socket> socket,
                                uint32_t pktSize,
                                uint32_t pktCount,
                                Time pktInterval);
};

UavVehicleNetwork::UavVehicleNetwork() {
    interval = 1;
    interPacketInterval = Seconds(interval);
    phyMode = std::string("DsssRate1Mbps");

    // Config::SetDefault("ns3::WifiRemoteStationManager::ConstantRateWifiManager",
    //                    StringValue(phyMode));
    seed = 1;
    RngSeedManager::SetSeed(seed);
    uv = CreateObject<UniformRandomVariable>();
    stopTime = 110;
    tracing = true;
    packetSize = 1024;
    nodeNumber = 40;
    distance = 25.0;
    startTime = 2.0;
    height = 3;
    sarNumber = 20;
    transDistance = 25;
    priority = 0;
    serverPort = 10000;

    mspeed = 1.0;

    clientStopTime = 200;
    serverStopTime = 200;
    clientStartTime = 2;
    serverStartTime = 1;

    sendNum = 0;
    recvNum = 0;
    lostNum = 0;
    lossRatio = 0;
    deliveryRatio = 0;
    ETEDelay = 0;
    jitterDelay = 0;
    avgThroughput = 0;
    usingDsr = false;
}

UavVehicleNetwork::~UavVehicleNetwork() {}

void UavVehicleNetwork::CreateNodes() {
    nodes.Create(nodeNumber);

    MobilityHelper mobility;
    mobility.SetPositionAllocator(
        "ns3::RandomBoxPositionAllocator", "X",
        StringValue("ns3::UniformRandomVariable[Min=0|Max=50]"), "Y",
        StringValue("ns3::UniformRandomVariable[Min=0|Max=50]"), "Z",
        StringValue("ns3::UniformRandomVariable[Min=0|Max=5]"));

    /* 设置固定速度模型 */
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    for (size_t i = 0; i < nodeNumber / 2; i++) {
        mobility.Install(nodes.Get(i));
        Ptr<ConstantVelocityMobilityModel> mob =
            nodes.Get(i)->GetObject<ConstantVelocityMobilityModel>();
        double x = uv->GetInteger(0, 50);
        double y = uv->GetInteger(0, 50);
        double xSpeed = uv->GetValue(1, 3);
        double ySpeed = uv->GetValue(1, 3);
        mob->SetPosition(Vector(x, y, 1.0));
        mob->SetVelocity(Vector(xSpeed, ySpeed, 0));
    }

    mspeed = 1.0;
    /* 随机游走模型 */
    mobility.SetMobilityModel(
        "ns3::RandomWalk2dMobilityModel", "Mode", StringValue("Distance"),
        "Distance", StringValue("5"), "Speed",
        StringValue("ns3::ConstantRandomVariable[Constant=" +
                    std::to_string(mspeed) + "]"),
        "Bounds", StringValue("0|50|0|50"));
    for (uint32_t i = nodeNumber / 2; i < nodes.GetN(); i++) {
        mobility.Install(nodes.Get(i));
    }
}

void UavVehicleNetwork::CreateDevices() {
    WifiHelper wifi;

    /* 设置wifi物理信息 */
    /* 设置接受增益 */
    wifiPhy.Set("RxGain", DoubleValue(-10));
    wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);

    // wifi.SetRemoteStationManager(ns3::WifiRemoteStationManager);
    YansWifiChannelHelper wifiChannel;
    /* 设置传播时延 */
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    /* 设置路径损失和最大传输距离 */
    wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange",
                                   DoubleValue(transDistance));
    wifiPhy.SetChannel(wifiChannel.Create());
    WifiMacHelper wifiMac;

    /* 设置wifi标准 */
    // wifi.SetStandard(WIFI_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
                                 StringValue("OfdmRate6Mbps"),
                                 "RtsCtsThreshold", UintegerValue(0));
    /* 设置自组网物理地址模型 */
    wifiMac.SetType("ns3::AdhocWifiMac");
    devices = wifi.Install(wifiPhy, wifiMac, nodes);

    if (tracing) {
        // LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
        // LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);

        AsciiTraceHelper ascii;
        wifiPhy.EnableAsciiAll(ascii.CreateFileStream("flying-adhoc.tr"));
        wifiPhy.EnablePcap("flying-adhoc", devices);
    }
}

void UavVehicleNetwork::InstallInternetStack() {
    /* 设置路由协议 */
    /* 设置不同协议的优先级 */
    /* 设置网络协议栈,list中的路由协议已经存在list中 */
    DsrMainHelper dsrMain;
    DsrHelper dsr;
    if (usingDsr) {
        internet.Install(nodes);
        dsrMain.Install(dsr, nodes);
    } else {
        internet.SetRoutingHelper(list);
        internet.Install(nodes);
    }

    Ipv4AddressHelper ipv4;
    NS_LOG_INFO("Assign IP Addresses.");
    /* 设置网段和子网掩码 */
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    /* 为网卡分配ipv4地址，并保存 */
    interface = ipv4.Assign(devices);
}

void UavVehicleNetwork::InstallApplications() {
    /* 服务器接收到客户端udp数据包后，会向客户端发送同一个包 */

    /* 设置服务器 */
    UdpEchoServerHelper echoServer(serverPort);
    ApplicationContainer serverApps = echoServer.Install(nodes.Get(server));
    serverApps.Start(Seconds(serverStartTime));
    serverApps.Stop(Seconds(stopTime));

    /* 配置客户端信息 */
    UdpEchoClientHelper echoClient(interface.GetAddress(server), serverPort);
    echoClient.SetAttribute("MaxPackets", UintegerValue(MaxPackets));
    echoClient.SetAttribute("Interval", TimeValue(interPacketInterval));
    echoClient.SetAttribute("PacketSize", UintegerValue(packetSize));

    // NodeContainer clientNodes;
    /* 安装客户端程序 */
    for (uint32_t i = 0; i < client.size(); i++) {
        ApplicationContainer clientApps =
            echoClient.Install(nodes.Get(client[i]));
        clientApps.Start(Seconds(clientStartTime));
        clientApps.Stop(Seconds(stopTime));
    }

    Simulator::Stop(Seconds(stopTime));

    /* 放在之前Run之前，否则报错 */
    AnimationInterface anim("flying-ad-hoc.xml");  // Mandatory
    anim.SetMaxPktsPerTraceFile(INT64_MAX);
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        anim.UpdateNodeDescription(nodes.Get(i),
                                   std::to_string(i));  // Optional
        anim.UpdateNodeColor(nodes.Get(i), 255, 0, 0);  // Optional
    }

    NodeContainer flowNodes;
    FlowMonitorHelper flowmon;

    /* 只对客户端和服务端进行监听 */
    flowNodes.Add(nodes.Get(server));
    for (size_t i = 0; i < client.size(); i++) {
        flowNodes.Add(nodes.Get(client[i]));
    }

    Ptr<FlowMonitor> monitor = flowmon.Install(flowNodes);

    /* 对全部节点监听 */
    // Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    // monitor->SetAttribute("FlowInterruptionsMinTime",
    // TimeValue(Seconds(1.0)));

    // monitor->SetAttribute("MaxPerHopDelay", TimeValue(Seconds(15.0)));

    Simulator::Run();

    uint32_t SentPackets = 0;
    uint32_t ReceivedPackets = 0;
    uint32_t LostPackets = 0;

    int flowNumber = 0;
    float AvgThroughput = 0;
    Time Jitter;
    Time Delay;

    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter =
             stats.begin();
         iter != stats.end(); ++iter) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(iter->first);

        /* 添加判定条件，监听指定的IP地址和端口的包 */
        if (FlowCondition(t)) {
            uint32_t rxPackets = iter->second.rxPackets;
            NS_LOG_UNCOND("----Flow ID:" << iter->first);
            NS_LOG_UNCOND("Src Addr" << t.sourceAddress << "\tDst Addr "
                                     << t.destinationAddress);
            /* 端口号 */
            NS_LOG_UNCOND("Src Port " << t.sourcePort << "\tDst Port "
                                      << t.destinationPort);

            NS_LOG_UNCOND("Sent Packets= " << iter->second.txPackets);
            NS_LOG_UNCOND("Received Packets = " << rxPackets);
            NS_LOG_UNCOND("Lost Packets = " << iter->second.lostPackets);
            NS_LOG_UNCOND("Packet delivery ratio = "
                          << rxPackets * 100.0 / iter->second.txPackets << "%");
            NS_LOG_UNCOND("Packet loss ratio = "
                          << (iter->second.lostPackets * 100.0) /
                                 iter->second.txPackets
                          << "%");
            if (rxPackets != 0) {
                NS_LOG_UNCOND("Delay = "
                              << iter->second.delaySum.GetMilliSeconds() /
                                     rxPackets
                              << " ms");
                NS_LOG_UNCOND("Jitter = "
                              << iter->second.jitterSum.GetMilliSeconds() /
                                     rxPackets
                              << " ms");
            } else {
                NS_LOG_UNCOND("delaySum: "
                              << iter->second.delaySum.GetMilliSeconds()
                              << " ms");
                NS_LOG_UNCOND("Jitter = "
                              << iter->second.jitterSum.GetMilliSeconds()
                              << " ms");
            }
            NS_LOG_UNCOND("Throughput = "
                          << iter->second.rxBytes * 8.0 /
                                 (iter->second.timeLastRxPacket.GetSeconds() -
                                  iter->second.timeFirstTxPacket.GetSeconds()) /
                                 1024
                          << " Kbps");
            NS_LOG_UNCOND("received size = " << iter->second.rxBytes << "bits");

            SentPackets = SentPackets + (iter->second.txPackets);
            ReceivedPackets = ReceivedPackets + (iter->second.rxPackets);
            LostPackets = LostPackets + iter->second.lostPackets;
            AvgThroughput =
                AvgThroughput + (iter->second.rxBytes * 8.0 /
                                 (iter->second.timeLastRxPacket.GetSeconds() -
                                  iter->second.timeFirstTxPacket.GetSeconds()) /
                                 1024);

            /* 忽略了时延为0的包，即丢弃的包 */
            /* 这个计算的是所有的客户端到服务端的时延 */
            /* Delay记录每个链路每个包的总时延 */
            /* Jitter记录每个链路每个包的抖动之和 */
            Delay = Delay + (iter->second.delaySum);
            Jitter = Jitter + (iter->second.jitterSum);

            /* 统计的流的数量 */
            flowNumber = flowNumber + 1;
        }
    }

    if (flowNumber != 0) {
        AvgThroughput = AvgThroughput / flowNumber;
    }

    NS_LOG_UNCOND("--------Total Results of the simulation----------"
                  << std::endl);
    NS_LOG_UNCOND("Total sent packets = " << SentPackets);
    NS_LOG_UNCOND("Total Received Packets = " << ReceivedPackets);
    NS_LOG_UNCOND("Total Lost Packets = " << LostPackets);
    if (SentPackets != 0) {
        NS_LOG_UNCOND("Packet Loss ratio = "
                      << ((LostPackets * 100.0) / SentPackets) << " %");
        NS_LOG_UNCOND("Packet delivery ratio = "
                      << ((ReceivedPackets * 100.0) / SentPackets) << " %");
    }

    NS_LOG_UNCOND("Average Throughput = " << AvgThroughput << " Kbps");

    if (ReceivedPackets != 0) {
        NS_LOG_UNCOND("End to End Delay = "
                      << Delay.GetMilliSeconds() / ReceivedPackets << " ms");
        NS_LOG_UNCOND("End to End Jitter delay = "
                      << Jitter.GetMilliSeconds() / ReceivedPackets << " ms");
    }

    NS_LOG_UNCOND("Total Flod id " << flowNumber);

    sendNum = SentPackets;
    recvNum = ReceivedPackets;
    lostNum = LostPackets;
    if (SentPackets != 0) {
        lossRatio = (LostPackets * 100.0) / SentPackets;
        deliveryRatio = (ReceivedPackets * 100.0) / SentPackets;
    }
    if (ReceivedPackets != 0) {
        ETEDelay = Delay.GetMilliSeconds() / ReceivedPackets;
        jitterDelay = Jitter.GetMilliSeconds() / ReceivedPackets;
    }

    avgThroughput = AvgThroughput;
    monitor->SerializeToXmlFile("manet-routing.xml", true, true);
    Simulator::Destroy();
    std::cout << "Simulator Finished" << std::endl;
}

void UavVehicleNetwork::SetArgument(
    const std::map<std::string, std::string>& arguments) {
    std::map<std::string, std::string>::const_iterator iter;
    iter = arguments.begin();
    /* 开始设置参数 */
    for (; iter != arguments.end(); iter++) {
        if (iter->first == "tracing") {
            if (iter->second == std::string("true") ||
                iter->second == std::string("1")) {
                tracing = true;
            } else {
                tracing = false;
            }
            continue;
        }
        /* 设置包的大小 */
        if (iter->first == "packetSize") {
            packetSize = std::stoi(iter->second);
            continue;
        }
        /* 设置发送的最大的包 */
        if (iter->first == "MaxPackets") {
            MaxPackets = std::stoi(iter->second);
            continue;
        }
        /* 设置每个包的间隔 */
        if (iter->first == "interval") {
            interval = std::stod(iter->second);
            continue;
        }
        /* 使用Socket发送包时：设置停止时间 */
        if (iter->first == "stopTime") {
            stopTime = std::stod(iter->second);
            continue;
        }
        /* 设置发送的包的最大数量 */
        if (iter->first == "MaxPackets") {
            MaxPackets = std::stoi(iter->second);
            continue;
        }
        /* 设置节点数量 */
        if (iter->first == "nodeNumber") {
            nodeNumber = std::stoi(iter->second);
            continue;
        }
        /* 设置客户端的数量 */
        if (iter->first == "sarNumber") {
            sarNumber = std::stoi(iter->second);
            continue;
        }
        /* 设置每个节点传输距离 */
        if (iter->first == "transDistance") {
            transDistance = std::stod(iter->second);
            continue;
        }
        /* 设置随机种子 */
        if (iter->first == "seed") {
            seed = std::stoi(iter->second);
            RngSeedManager::SetSeed(seed);
            uv = CreateObject<UniformRandomVariable>();
            continue;
        }
    }
}

void UavVehicleNetwork::SetRoutingHelper(const Ipv4RoutingHelper& routing) {
    // list = (Ipv4ListRoutingHelper)routing;
    list.Add(routing, 10);
    // internet.SetRoutingHelper(routing);
}

void UavVehicleNetwork::SetDsr() {
    CreateNodes();
    CreateDevices();

    /* 设置路由协议 */

    DsrMainHelper dsrMain;
    DsrHelper dsr;
    internet.Install(nodes);
    dsrMain.Install(dsr, nodes);

    Ipv4AddressHelper ipv4;
    // NS_LOG_INFO("Assign IP Addresses.");
    /* 设置网段和子网掩码 */
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    /* 为网卡分配ipv4地址，并保存 */
    interface = ipv4.Assign(devices);

    int dsrServer = uv->GetInteger(0, nodeNumber - 1);
    int dsrClient = uv->GetInteger(0, nodeNumber - 1);

    while (dsrClient == dsrServer) {
        dsrClient = uv->GetInteger(0, nodeNumber - 1);
    }

    dsrServer = 0;
    dsrClient = nodeNumber - 1;

    std::cout << "DSR server: " << dsrServer << std::endl;
    std::cout << "DSR client: " << dsrClient << std::endl;

    InstallSocket(dsrServer, dsrClient);
}

void UavVehicleNetwork::SetUsingDsr(bool usingDsr) {
    this->usingDsr = usingDsr;
}

void UavVehicleNetwork::InstallSocket(int server, int client) {
    /* 设置Socket的类型 */
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    /* 创建Socket */
    Ptr<Socket> recvSink = Socket::CreateSocket(nodes.Get(server), tid);
    /* 配置服务端的接受地址 */
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);
    /* 绑定socket和地址 */
    recvSink->Bind(local);
    /* 设置收到包的回调函数 */
    recvSink->SetRecvCallback(MakeCallback(&UavVehicleNetwork::ReceivePacket));

    /* 开始客户端的配置：负责发送数据 */
    Ptr<Socket> source = Socket::CreateSocket(nodes.Get(client), tid);
    InetSocketAddress remote =
        InetSocketAddress(interface.GetAddress(server, 0), 80);
    source->Connect(remote);

    /* 开始时间 */
    Simulator::Schedule(Seconds(startTime), &UavVehicleNetwork::GenerateTraffic,
                        source, packetSize, MaxPackets, interPacketInterval);
    Simulator::Stop(Seconds(stopTime));
    Simulator::Run();
    Simulator::Destroy();

    /* 开始获取网络性能数据 */
    sendNum = sendPacketNumber;
    recvNum = UavVehicleNetwork::recvPacketNumber;
    lostNum = sendNum - recvNum;
    deliveryRatio = 100.0 * recvNum / sendNum;
    lossRatio = 100.0 * lostNum / sendNum;
    /* 开始对时延进行处理 */
    int sumDelay = 0;
    int sumJitter = 0;
    for (size_t i = 0; i < delay.size(); i++) {
        sumDelay += delay[i];
        if (i == 0) {
            continue;
        } else {
            sumJitter += abs(delay[i] - delay[i - 1]);
        }
    }
    if (recvNum != 0) {
        ETEDelay = sumDelay / recvNum;
        jitterDelay = sumJitter / recvNum;
        avgThroughput = 1000 * (recvByte * 8.0) /
                        ((rxLastPacketTime - txFirstPacketTime) * 1000);
    }
}

void UavVehicleNetwork::SetSimulator(Ptr<SimulatorImpl> simulator) {
    Simulator::SetImplementation(simulator);
}

void UavVehicleNetwork::SetupEnvironment() {
    CreateNodes();
    CreateDevices();
    InstallInternetStack();
}

void UavVehicleNetwork::Run() {
    /* 使用udp客户端发送包 */
    server = uv->GetInteger(0, nodeNumber - 1);
    // server = 22;

    std::cout << "server: " << server << std::endl;
    for (int i = 0; i < sarNumber; i++) {
        int value = uv->GetInteger(0, nodeNumber - 1);
        /* 防止客户端和服务端在同一个节点 */
        while (value == server) {
            value = uv->GetInteger(0, nodeNumber - 1);
        }
        client.push_back(value);
        std::cout << "client: " << client[i] << std::endl;
    }
    /* 使用DSR的使用Socket发送数据 */
    /* 使用Socket发送UDP数据，统计网络性能时，只能统计单个节点发送和接受 */
    if (usingDsr) {
        resetResult();
        InstallSocket(server, client[0]);
    } else {
        /* 使用UDP服务端发送的时候，使用的是多对一 */
        InstallApplications();
    }
}

/**
 * 获取结果：平均时延、丢包率
 */
std::map<std::string, std::string>* UavVehicleNetwork::GetResult() {
    std::cout << std::endl;
    std::cout << "my result---------------------------" << std::endl;
    std::cout << "send packet: " << sendNum << std::endl;
    std::cout << "received packet: " << recvNum << std::endl;
    std::cout << "lost packet: " << lostNum << std::endl;
    std::cout << "Deliver Rate: " << deliveryRatio << " %" << std::endl;
    std::cout << "Loss Rate: " << lossRatio << " %" << std::endl;
    std::cout << "received Byte: " << recvByte << " Byte" << std::endl;
    std::cout << "trasTime: " << (rxLastPacketTime - txFirstPacketTime) << " ms"
              << std::endl;

    std::cout << "end to end delay: " << ETEDelay << " ms" << std::endl;
    std::cout << "end to end jitter delay: " << jitterDelay << " ms"
              << std::endl;
    std::cout << "average throughput: " << avgThroughput << " kbps"
              << std::endl;
    std::cout << std::endl;

    result.insert(std::pair<std::string, std::string>("sendNum",
                                                      std::to_string(sendNum)));
    result.insert(std::pair<std::string, std::string>("recvNum",
                                                      std::to_string(recvNum)));
    result.insert(std::pair<std::string, std::string>("lostNum",
                                                      std::to_string(lostNum)));
    result.insert(std::pair<std::string, std::string>(
        "deliveryRatio", std::to_string(deliveryRatio)));
    result.insert(std::pair<std::string, std::string>(
        "lossRatio", std::to_string(lossRatio)));
    result.insert(std::pair<std::string, std::string>(
        "ETEDelay", std::to_string(ETEDelay)));
    result.insert(std::pair<std::string, std::string>(
        "jitterDelay", std::to_string(jitterDelay)));
    result.insert(std::pair<std::string, std::string>(
        "avgThroughput", std::to_string(avgThroughput)));
    return &result;
}

/* 过滤监听的UDP数据包的条件 */
bool UavVehicleNetwork::FlowCondition(Ipv4FlowClassifier::FiveTuple t) {
    /* 从服务端到客户端 */
    ns3::Ipv4Address serverAddress = interface.GetAddress(server);
    for (size_t i = 0; i < client.size(); i++) {
        ns3::Ipv4Address clientAddress = interface.GetAddress(client[i]);
        /* 从服务端到客户端或者从客户端到服务端 */
        if (/* (t.sourceAddress == serverAddress &&
             t.destinationAddress == clientAddress) || */
            (t.sourceAddress == clientAddress &&
             t.destinationAddress == serverAddress)) {
            /* 开始指定端口 */
            if (t.sourcePort == serverPort || t.destinationPort == serverPort) {
                return true;
            }
            return false;
        }
    }
    return false;
}

void UavVehicleNetwork::resetResult() {
    recvPacketNumber = 0;
    sendPacketNumber = 0;
    recvByte = 0;
    txFirstPacketTime = INT64_MAX;
    rxLastPacketTime = INT64_MIN;
    delay.clear();
}

/* 静态变量初始化 */
uint32_t UavVehicleNetwork::recvPacketNumber = 0;
uint32_t UavVehicleNetwork::sendPacketNumber = 0;
uint32_t UavVehicleNetwork::recvByte = 0;
int64_t UavVehicleNetwork::txFirstPacketTime = INT64_MAX;
int64_t UavVehicleNetwork::rxLastPacketTime = INT64_MIN;
std::vector<int64_t> UavVehicleNetwork::delay;

void UavVehicleNetwork::GenerateTraffic(Ptr<Socket> socket,
                                        uint32_t pktSize,
                                        uint32_t pktCount,
                                        Time pktInterval) {
    if (pktCount > 0) {
        Ptr<Packet> pack = Create<Packet>(pktSize);
        DelayJitterEstimation delayJitter;
        delayJitter.PrepareTx(pack);
        socket->Send(pack);
        UavVehicleNetwork::sendPacketNumber++;
        if (txFirstPacketTime == INT64_MAX) {
            UavVehicleNetwork::txFirstPacketTime =
                Simulator::Now().GetMilliSeconds();
        }
        Simulator::Schedule(pktInterval, &UavVehicleNetwork::GenerateTraffic,
                            socket, pktSize, pktCount - 1, pktInterval);
    } else {
        socket->Close();
    }
}

void UavVehicleNetwork::ReceivePacket(Ptr<Socket> socket) {
    Ptr<Packet> package;
    Address address;
    while ((package = socket->RecvFrom(address))) {
        // NS_LOG_UNCOND("Received one packet!");
        UavVehicleNetwork::recvPacketNumber++;
        DelayJitterEstimation delayJitter;
        delayJitter.RecordRx(package);
        Time t = delayJitter.GetLastDelay();
        int64_t nowTime = Simulator::Now().GetMilliSeconds();
        if (rxLastPacketTime < nowTime) {
            rxLastPacketTime = nowTime;
        }
        recvByte += package->GetSize();
        delay.push_back(t.GetMilliSeconds());
        std::cout << "current time:"
                  << "\t" << Simulator::Now().GetSeconds() << "\t"
                  << "Delay: "
                  << "\t" << t.GetMilliSeconds() << " ms";
        if (InetSocketAddress::IsMatchingType(address)) {
            InetSocketAddress addr = InetSocketAddress::ConvertFrom(address);
            std::cout << " received one packet from " << addr.GetIpv4();
        } else {
            std::cout << " received one packet!";
        }
        std::cout << std::endl;
    }
}

int main(int argc, char* argv[]) {
    OlsrHelper olsr;
    AodvHelper aodv;
    DsdvHelper dsdv;
    GpsrHelper gpsr;
    BatmandHelper batman;
    Ipv4StaticRoutingHelper staticRouting;
    UavVehicleNetwork test;
    std::map<std::string, std::string> arguments;
    arguments.insert(std::pair<std::string, std::string>("sarNumber", "20"));
    arguments.insert(
        std::pair<std::string, std::string>("MaxPackets", "10"));
    arguments.insert(
        std::pair<std::string, std::string>("stopTime", " 100"));
    arguments.insert(
        std::pair<std::string, std::string>("tracing", "true"));
    arguments.insert(
        std::pair<std::string, std::string>("transDistance", "25"));
    arguments.insert(
        std::pair<std::string, std::string>("packetSize", "1024"));
    arguments.insert(
        std::pair<std::string, std::string>("seed", "100"));
    test.SetArgument(arguments);
    test.SetUsingDsr(false);
    test.SetRoutingHelper(aodv);
    test.SetupEnvironment();
    test.Run();

    std::map<std::string, std::string>* result = test.GetResult();

    std::map<std::string, std::string> res = *result;
    std::map<std::string, std::string>::const_iterator iter;
    iter = res.begin();

    /* 开始设置参数 */
    std::cout << "print result----------------" << std::endl;
    for (; iter != res.end(); iter++) {
        std::cout << iter->first << "   " << iter->second << std::endl;
    }
    return 0;
}