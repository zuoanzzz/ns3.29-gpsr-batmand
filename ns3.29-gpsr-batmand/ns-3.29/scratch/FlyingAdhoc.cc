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
#include "ns3/buildings-module.h"

#include "ns3/aodv-helper.h"
#include "ns3/dsdv-helper.h"
#include "ns3/dsr-helper.h"
#include "ns3/olsr-helper.h"
#include "ns3/gpsr-helper.h"
#include "ns3/batmand-helper.h"

#include "ns3/dsr-main-helper.h"

#include "ns3/flow-monitor-module.h"

#include "base-network.h"

#include "ns3/SpiralModel.h"
#include "ns3/circle-mobility.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("FlyingAdHoc");

class FlyingAdHoc{
   private:
    NodeContainer nodes;
    NetDeviceContainer devices;
    std::string phyMode;
    Ipv4InterfaceContainer interface;
    Ipv4ListRoutingHelper list;
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    
    Ipv4StaticRoutingHelper staticRouting;
    
    InternetStackHelper internet;

    uint32_t packetSize; /* 每个包的大小 */
    uint32_t maxPackets; /* 每个节点发送的包的数量 */
    double interval;     /* 发送每个包的时间间隔 */
    double simulatorEndTime;     /* 设置网络模拟停止时间 */
    uint32_t nWifi; /* 节点的总数量 */
    double distance;
    double startTime;     /* 开始模拟时间 */
    int appNum;        /* 发送和接受节点的数量 */
    //int seed;             /* 设置随机种子 */
    double mspeed; /* 设置节点速度：除了匀速运动和高斯运动 */

    Time interPacketInterval; /* 发送包的时间间隔 */

    int priority; /* 每个协议的优先级 */
    uint32_t appServerPort;

    int client;
    int server;

    double appClientBeginTime;
    double appServerBeginTime;

    double appClientEndTime;
    double appServerEndTime;

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
    FlyingAdHoc();
    ~FlyingAdHoc();

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
  double rad(int a,int b,int r){
  double R ;
  srand(r);//设置随机数种子，使每次产生的随机序列不同
		 R = rand() % (b*100) / double(100);
		R+=a;
  return R;
}
};

FlyingAdHoc::FlyingAdHoc() {
    interval = 1;
    interPacketInterval = Seconds(interval);
    phyMode = std::string("DsssRate1Mbps");

    // Config::SetDefault("ns3::WifiRemoteStationManager::ConstantRateWifiManager",
    //                    StringValue(phyMode));

    simulatorEndTime = 110;
    packetSize = 1024;
    maxPackets = 10;
    nWifi = 30;
    distance = 5.0;
    startTime = 2.0;
    appNum = 5;
    appServerPort = 10000;

    mspeed = 1.0;

    appServerEndTime = 100;
    appServerEndTime = 100;
    appClientBeginTime = 20;
    appServerBeginTime = 15;

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

FlyingAdHoc::~FlyingAdHoc() {}

void FlyingAdHoc::CreateNodes() {
    nodes.Create(nWifi);

    MobilityHelper mobility;
    mobility.SetPositionAllocator(
        "ns3::RandomBoxPositionAllocator", "X",
        StringValue("ns3::UniformRandomVariable[Min=0|Max=50]"), "Y",
        StringValue("ns3::UniformRandomVariable[Min=0|Max=50]"), "Z",
        StringValue("ns3::UniformRandomVariable[Min=0|Max=5]"));


    mobility.SetMobilityModel ("ns3::GaussMarkovMobilityModel", "Bounds",
                             BoxValue (Box (0, 10, 0, 10, 0, 10)), "TimeStep",
                             TimeValue (Seconds (0.5)), "Alpha", DoubleValue (0.85));
    /* 车辆：固定速度模型 */
  for (size_t i = nWifi/2; i < nWifi; i++)
    {
      mobility.Install (nodes.Get (i));
      Ptr<ConstantVelocityMobilityModel> mod = nodes.Get (i)->GetObject<ConstantVelocityMobilityModel> ();
      double x=rad(0,10,i);
      double y=rad(0,10,i+10);
      double xs=rad(0,1,i);
      double ys=rad(0,1,i+110);
      mod->SetPosition (Vector (x,y, 1.0));
      mod->SetVelocity (Vector (xs,ys, 0));
    }

    mspeed = 1.0;
    /* 无人机：随机游走模型 */
    mobility.SetMobilityModel(
        "ns3::RandomWalk2dMobilityModel", "Mode", StringValue("Distance"),
        "Distance", StringValue("5"), "Speed",
        StringValue("ns3::ConstantRandomVariable[Constant=" +
                    std::to_string(mspeed) + "]"),
        "Bounds", StringValue ("0|50|0|50"));
    for (uint32_t i = nWifi/2; i < nWifi; i++) {
        mobility.Install(nodes.Get(i));
    }

    BuildingsHelper::Install (nodes);
}

void FlyingAdHoc::CreateDevices() {
    WifiHelper wifi;
    // wifi.SetRemoteStationManager(ns3::WifiRemoteStationManager);
    YansWifiChannelHelper wifiChannel;
    wifiPhy.SetChannel(wifiChannel.Create());
    WifiMacHelper wifiMac;
    /* 设置自组网物理地址模型 */
    wifiMac.SetType("ns3::AdhocWifiMac");
    devices = wifi.Install(wifiPhy, wifiMac, nodes);

}

void FlyingAdHoc::InstallInternetStack() {
    /* 设置路由协议 */
    /* 设置不同协议的优先级 */
    /* 设置网络协议栈,list中的路由协议已经存在list中 */
    DsrMainHelper dsrMain;
    DsrHelper dsr;
        internet.Install(nodes);
        dsrMain.Install(dsr, nodes);
    Ipv4AddressHelper ipv4;
    NS_LOG_INFO("Assign IP Addresses.");
    /* 设置网段和子网掩码 */
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    /* 为网卡分配ipv4地址，并保存 */
    interface = ipv4.Assign(devices);
}

void FlyingAdHoc::SetArgument(const std::map<std::string, std::string> &arguments){
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

void FlyingAdHoc::SetUsingDsr(bool usingDsr) {
    this->usingDsr = usingDsr;
}

void FlyingAdHoc::InstallSocket(int server, int client) {
    /* 设置Socket的类型 */
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    /* 创建Socket */
    Ptr<Socket> recvSink = Socket::CreateSocket(nodes.Get(server), tid);
    /* 配置服务端的接受地址 */
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);
    /* 绑定socket和地址 */
    recvSink->Bind(local);
    /* 设置收到包的回调函数 */
    recvSink->SetRecvCallback(MakeCallback(&FlyingAdHoc::ReceivePacket));

    /* 开始客户端的配置：负责发送数据 */
    Ptr<Socket> source = Socket::CreateSocket(nodes.Get(client), tid);
    InetSocketAddress remote =
        InetSocketAddress(interface.GetAddress(server, 0), 80);
    source->Connect(remote);

    /* 开始时间 */
    Simulator::Schedule(Seconds(startTime), &FlyingAdHoc::GenerateTraffic,
                        source, packetSize, maxPackets, interPacketInterval);
    Simulator::Stop(Seconds(simulatorEndTime));
    Simulator::Run();
    Simulator::Destroy();

    /* 开始获取网络性能数据 */
    sendNum = sendPacketNumber;
    recvNum = FlyingAdHoc::recvPacketNumber;
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

void FlyingAdHoc::SetSimulator(Ptr<SimulatorImpl> simulator) {
    Simulator::SetImplementation(simulator);
}

void FlyingAdHoc::SetupEnvironment() {
    CreateNodes();
    CreateDevices();
    InstallInternetStack();
}

void FlyingAdHoc::Run() {
    /* 使用DSR的使用Socket发送数据 */
    /* 使用Socket发送UDP数据，统计网络性能时，只能统计单个节点发送和接受 */
    if (usingDsr) {
        resetResult();
        InstallSocket(4, 14);
    } else {
        /* 使用UDP服务端发送的时候，使用的是多对一 */
        //InstallApplications();
    }
}

/**
 * 获取结果：平均时延、丢包率
 */
std::map<std::string, std::string>* FlyingAdHoc::GetResult() {
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
bool FlyingAdHoc::FlowCondition(Ipv4FlowClassifier::FiveTuple t) {
    /* 从服务端到客户端 */
    ns3::Ipv4Address serverAddress = interface.GetAddress(server);
    for (int i = 0; i < client; i++) {
        ns3::Ipv4Address clientAddress = interface.GetAddress(i);
        /* 从服务端到客户端或者从客户端到服务端 */
        if (/* (t.sourceAddress == serverAddress &&
             t.destinationAddress == clientAddress) || */
            (t.sourceAddress == clientAddress &&
             t.destinationAddress == serverAddress)) {
            /* 开始指定端口 */
            if (t.sourcePort == appServerPort || t.destinationPort == appServerPort) {
                return true;
            }
            return false;
        }
    }
    return false;
}

void FlyingAdHoc::resetResult() {
    recvPacketNumber = 0;
    sendPacketNumber = 0;
    recvByte = 0;
    txFirstPacketTime = INT64_MAX;
    rxLastPacketTime = INT64_MIN;
    delay.clear();
}

/* 静态变量初始化 */
uint32_t FlyingAdHoc::recvPacketNumber = 0;
uint32_t FlyingAdHoc::sendPacketNumber = 0;
uint32_t FlyingAdHoc::recvByte = 0;
int64_t FlyingAdHoc::txFirstPacketTime = INT64_MAX;
int64_t FlyingAdHoc::rxLastPacketTime = INT64_MIN;
std::vector<int64_t> FlyingAdHoc::delay;

void FlyingAdHoc::GenerateTraffic(Ptr<Socket> socket,
                                  uint32_t pktSize,
                                  uint32_t pktCount,
                                  Time pktInterval) {
    if (pktCount > 0) {
        Ptr<Packet> pack = Create<Packet>(pktSize);
        DelayJitterEstimation delayJitter;
        delayJitter.PrepareTx(pack);
        socket->Send(pack);
        FlyingAdHoc::sendPacketNumber++;
        if (txFirstPacketTime == INT64_MAX) {
            FlyingAdHoc::txFirstPacketTime = Simulator::Now().GetMilliSeconds();
        }
        Simulator::Schedule(pktInterval, &FlyingAdHoc::GenerateTraffic, socket,
                            pktSize, pktCount - 1, pktInterval);
    } else {
        socket->Close();
    }
}

void FlyingAdHoc::ReceivePacket(Ptr<Socket> socket) {
    Ptr<Packet> package;
    Address address;
    while ((package = socket->RecvFrom(address))) {
        NS_LOG_UNCOND("Received one packet!");
        FlyingAdHoc::recvPacketNumber++;
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
                  << "\t" << t.GetMilliSeconds() << " ms" << std::endl;
    }
}

int main(int argc, char* argv[]) {
    FlyingAdHoc test;
    OlsrHelper olsr;
    AodvHelper aodv;
    DsdvHelper dsdv;
    GpsrHelper gpsr;
    BatmandHelper batman;

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

    test.SetArgument(params);
    Ipv4ListRoutingHelper list;
    /* 优先级大的先使用 */

    test.SetUsingDsr(true);
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