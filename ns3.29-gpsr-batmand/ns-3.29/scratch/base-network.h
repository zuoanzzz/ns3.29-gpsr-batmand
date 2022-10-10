#include "ns3/ipv4-routing-helper.h"
#include "ns3/simulator-impl.h"

using namespace ns3;

class BaseNetwork
{
public:
  /**
   * 1.设定参数
   */
  virtual void SetArgument (const std::map<std::string, std::string> &arguments) = 0;
  /**
   * 2（可选）.设定路由协议
   * 注意：如果调用的话，就需要在初始化环境的时候SetRoutingHelper
   */
  virtual void SetRoutingHelper (const Ipv4RoutingHelper &routing,int priority) = 0;
  /**
   * 3（可选）.设定模拟器
   * 注意：如果没有调用的话，就直接使用默认的模拟器Simulator
   */
  virtual void SetSimulator (Ptr<SimulatorImpl> simulator) = 0;
  /**
   * 4.初始化环境
   */
  virtual void SetupEnvironment () = 0;
  /**
   * 5.运行
   */
  virtual void Run () = 0;
  /**
   * 6.获取仿真结果
   */
  virtual std::map<std::string, std::string>* GetResult () = 0;
};