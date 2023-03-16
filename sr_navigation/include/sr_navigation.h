#ifndef __SR_NAVIGATION_H_
#define __SR_NAVIGATION_H_

#include "ros/ros.h"
#include "ros/package.h"
#include "srv/work.h"  //消毒工作服务消息类型头文件
#include "srv/goSite.h"  //去站点服务消息类型头文件
#include "std_srvs/Empty.h"  //清理代价地图障碍层消息类型头文件
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "action/TaskAction.h"
#include "std_msgs/String.h"
#include "msg/esp32_disinfect.h" //esp32_disinfect_data话题消息类型头文件
#include "msg/Battory.h" //电池电量消息类型头文件
#include "yaml-cpp/yaml.h"  //yaml文件读写头文件
#include "fstream"
#include "signal.h"               //异常捕捉头文件

#include "dwa_local_planner/DWAPlannerConfig.h" //DWA动态参数头文件
#include "dynamic_reconfigure/client.h"    //动态调参头文件
#include "boost/function.hpp"
#define resul_successful 0            //任务成功
#define resul_cancel  1               //客户端取消任务
#define resul_Insufficient_power 2    //电量不足，服务端取消
#define resul_way_file_error  3    //路线文件错误
#define feedback_go 0                 //前往目标点
#define feedback_Insufficient_disinfectant 2            //消毒液不足
#define feedback_waitting_replace_disinfectant 5        //等待更换消毒液
#define feedback_Disinfectant_replaced_successfully 3   //消毒液更换成功
#define feedback_Disinfecting    4      //正在消毒

using namespace std;

class TaskActionServer
{
public:
    TaskActionServer(string name,int count):
    client("/move_base/DWAPlannerROS",
           boost::bind(&TaskActionServer::ConfigurationCallback,this,_1)),
    ac_Movebase("move_base",true),
    as_Task(node,name,boost::bind(&TaskActionServer::NewGoal,this,_1,count),false), //给回调函数声明多个形参
    action_name_(name) {
    //获取指定功能包绝对路径
    package_dir = ros::package::getPath("sr_navigation");
    //获取点位pose参数文件名件名称
    ros::param::get("~home_name",home_name);
    ros::param::get("~charge_name",charge_name);
    ros::param::get("~disinfection_name",disinfection_name);
    //点位数量初始化
    Target_Point_Number = 0;
    //巡航起始点位初始化
    goal_point = 0;
    //电量判断位初始化
    SufficientPower = true;
    //液位判断位初始化
    SufficientDisinfectant = true;
    //巡航标志位初始化
    goto_way = false;
    //动作抢占回调函数
    as_Task.registerPreemptCallback(boost::bind(&TaskActionServer::PreemptCallback,this));
    //消毒工作服务~客户端
    client_work = node.serviceClient<srv::work>("/work");
    //move_base代价地图障碍层清除服务~客户端
    client_clear_costmaps = node.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    //move_base代价地图未知区域清除服务~客户端
    client_clear_unknown_space = node.serviceClient<std_srvs::Empty>("/move_base/client_clear_unknown_space");
    //前往站点服务~服务端
    srv_go_site = node.advertiseService("/go_site",&TaskActionServer::GoSiteCallback,this);
    //消毒控制板数据~订阅者
    sub_esp32 = node.subscribe("/esp32_disinfect_data",100,&TaskActionServer::ESP32Callback,this);
    //电池电量话题~订阅者
    sub_charge = node.subscribe("/Bot_Battory",50,&TaskActionServer::ChargeCallback,this);
    //开启ActionServer~Task
    as_Task.start();
  }
  ~TaskActionServer();
  void ConfigurationCallback(const dwa_local_planner::DWAPlannerConfig &config);
  void ESP32Callback(const msg::esp32_disinfect::ConstPtr &msg);
  void ChargeCallback(const msg::Battory::ConstPtr &msg);
  bool GoSiteCallback(srv::goSite::Request &req,srv::goSite::Response &res);
  void NewGoal(const action::TaskGoalConstPtr &goal,int count_ );
  bool GetYamlParam(const std::string &wayname);
  void PreemptCallback();
  bool CheckState();
protected:
  ros::NodeHandle node;
  ros::Time end_time;
  ros::Subscriber sub_esp32,sub_charge;
  ros::ServiceClient client_work,client_clear_costmaps,client_clear_unknown_space;
  ros::ServiceServer srv_go_site;
  dynamic_reconfigure::Client<dwa_local_planner::DWAPlannerConfig> client;
  dwa_local_planner::DWAPlannerConfig dwa_config;
  std::vector<move_base_msgs::MoveBaseGoal> Goal_points;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_Movebase;
  actionlib::SimpleActionServer<action::TaskAction> as_Task;
  action::TaskGoalConstPtr GoalConstPtr;
  action::TaskGoalPtr GoalPtr = boost::make_shared<action::TaskGoal>();//智能指针声明及初始化;
  action::TaskResult resul_;
  action::TaskFeedback feedback_;
  srv::work work_;
  srv::goSite gosite_;
  std_srvs::Empty clear_;
  //boost::shared_ptr<action::TaskGoalConstPtr> GoalPtr = boost::make_shared<action::TaskGoalConstPtr>();//智能指针声明及初始化
  string action_name_,package_dir,home_name,charge_name,disinfection_name;    
  int Target_Point_Number,goal_point;
  bool SufficientDisinfectant,SufficientPower;
  bool goto_way;   //路线巡航标志位 
};
#endif