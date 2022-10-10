#include "base-network.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
class UavVehicleNetwork : public BaseNetwork
{
private:
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor;
  std::map<std::string, std::string> rs; //canshu
  uint32_t nWifi = 30;                // 节点个数
  Ipv4RoutingHelper* routing = NULL;  // 路由协议
  uint32_t maxPackets = 1;            // 发送包的最大数量
  double_t interval = 1.0;            // 数据包之间等待时长
  uint32_t packetSize = 1024;         // 数据包的有效负载

  

  Ipv4ListRoutingHelper list;
  Ipv4StaticRoutingHelper staticRouting;
  
  NodeContainer flowNodes;
  NodeContainer wifiAdhocNodes;
  double_t appServerBeginTime = 1.0;  // 应用层 服务器开始时间
  double_t appServerEndTime = 20.0;   // 应用层 服务器结束时间
  double_t appClientBeginTime = 2.0;  // 应用层 客户端开始时间
  double_t appClientEndTime = 20.0;   // 应用层 客户端结束时间
  double_t simulatorEndTime = 20.0;   // 模拟器结束时间
  Ipv4InterfaceContainer AdhocInterfaces;
  std::string baseIp = "10.1.1.0";         // 分配起始IP地址
  std::string subNetMask = "255.255.255.0";// 子网掩码

  uint16_t appServerPort = 10000;         // 应用层服务器初始端口号
  uint16_t appNum = 30;                   // 应用app数量


    std::vector<int> client;
     std::vector<int> server;

public:
double speed;
  UavVehicleNetwork ();
  ~UavVehicleNetwork ();
  /**
   * 1.设定参数
   */
  void SetArgument (const std::map<std::string, std::string> &arguments);
  /**
   * 2（可选）.设定路由协议
   * 注意：如果调用的话，就需要在初始化环境的时候SetRoutingHelper
   */
  void SetRoutingHelper (const Ipv4RoutingHelper &routing,int priority);
  /**
   * 3（可选）.设定模拟器
   * 注意：如果没有调用的话，就直接使用默认的模拟器Simulator
   */
  void SetSimulator (Ptr<SimulatorImpl> simulator);
  /**
   * 4.初始化环境
   */
  void SetupEnvironment ();
  /**
   * 5.运行
   */
  void Run ();
  /**
   * 6.获取仿真结果
   */
  std::map<std::string, std::string> *GetResult ();
};
