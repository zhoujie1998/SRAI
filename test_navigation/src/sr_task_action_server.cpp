/*
通过自定义action Task 得到任务路线名，工作模式，消毒模式，工作时间
再通过传目标点给move_base
根据消毒控制板ESP32上报的液位和电量数据来自动发布相应目标站点，
实现在任务中液位不足可自动前往加液站，加液完成后继续任务

#消毒液不足不会结束任务，可以中断任务，服务端反馈Feedback 2，前往加液站更换消毒液后，继续任务
#电量不足，服务端直接结束任务，返回result 2

# Define the goal
string RouteName     #路线名称
uint8 WorkMode       #巡航消毒：0  点位消毒：1
uint8 DisinfectMode  #左水箱：1  右水箱：2  双水箱：3
uint16 WorkTime      #单位为min
---
# Define the resul
uint8 Resul          #任务完成：0  用户取消：1  电量不足：2    其他：3
---
# Define a feedback message
uint8 Feedback       #前往目标：0  恢复行为：1  消毒液不足：2  消毒液更换完成：3  正在消毒：4


*/
#include "ros/ros.h"
#include "srv/work.h"
#include "std_srvs/Empty.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "action/TaskAction.h"
#include "std_msgs/Bool.h"
#include "msg/esp32_disinfect.h"
#include "signal.h"
#define resul_successful 0            //任务成功
#define resul_cancel  1               //客户端取消任务
#define resul_Insufficient_power 2    //电量不足，服务端取消
#define feedback_go 0                 //前往目标点
#define feedback_Insufficient_disinfectant 2            //消毒液不足
#define feedback_Disinfectant_replaced_successfully 3   //消毒液更换成功
#define feedback_Disinfecting    4      //正在消毒
#define ShuZhuChangDu 15
#define NUM 10             //路线最大点位数
using namespace std;

class TaskActionServer
{
public:
    TaskActionServer(string name):
    ac_Movebase("move_base",true),
    as_Task(node,name,boost::bind(&TaskActionServer::newGoal,this,_1),false),
    action_name_(name) {
    //电量判断位初始化
    isSufficientPower=true;
    //液位判断位初始化
    isSufficientDisinfectant=true;
    //加载路线点位数量
    node.getParam("/Target_Point_Number",Target_Point_Number);
    //加载站点点位信息
    Goal_home.target_pose.header.frame_id="map";
    node.getParam("/home/positionX",Goal_home.target_pose.pose.position.x);
    node.getParam("/home/positionY",Goal_home.target_pose.pose.position.y);
    node.getParam("/home/positionZ",Goal_home.target_pose.pose.position.z);
    node.getParam("/home/orientationX",Goal_home.target_pose.pose.orientation.x);
    node.getParam("/home/orientationY",Goal_home.target_pose.pose.orientation.y);
    node.getParam("/home/orientationZ",Goal_home.target_pose.pose.orientation.z);
    node.getParam("/home/orientationW",Goal_home.target_pose.pose.orientation.w);
    Goal_disinfection_station.target_pose.header.frame_id="map";
    node.getParam("/disinfection_station/positionX",Goal_disinfection_station.target_pose.pose.position.x);
    node.getParam("/disinfection_station/positionY",Goal_disinfection_station.target_pose.pose.position.y);
    node.getParam("/disinfection_station/positionZ",Goal_disinfection_station.target_pose.pose.position.z);
    node.getParam("/disinfection_station/orientationX",Goal_disinfection_station.target_pose.pose.orientation.x);
    node.getParam("/disinfection_station/orientationY",Goal_disinfection_station.target_pose.pose.orientation.y);
    node.getParam("/disinfection_station/orientationZ",Goal_disinfection_station.target_pose.pose.orientation.z);
    node.getParam("/disinfection_station/orientationW",Goal_disinfection_station.target_pose.pose.orientation.w);
    //as_Task中断处理回调函数
    as_Task.registerPreemptCallback(boost::bind(&TaskActionServer::preemptCB,this));
    //消毒工作模式服务
    client_work=node.serviceClient<srv::work>("/work");
    //move_base代价地图障碍层清除服务
    client_clear_costmaps=node.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    //move_base代价地图未知区域清除服务
    client_clear_unknown_space=node.serviceClient<std_srvs::Empty>("/move_base/client_clear_unknown_space");
    //回充电站话题发布者
    pub_gohome=node.advertise<std_msgs::Bool>("GoHome",100);
    //回充电站话题订阅者
    sub_go_home=node.subscribe("/GoHome",100,&TaskActionServer::GoHomeCallback,this);
    //回加液站话题订阅者
    sub_go_disinfection_station=node.subscribe("/GoDisinfectionStation",100,&TaskActionServer::GoDisinfectionStationCallback,this);
    //消毒控制版数据订阅者
    sub_ESP32=node.subscribe("esp32_disinfect_data",100,&TaskActionServer::ESP32Callback,this);
    //开启action服务端
    as_Task.start();
  }
  ~TaskActionServer();
  void ESP32Callback(const msg::esp32_disinfect::ConstPtr &msg);
  void GoHomeCallback(const std_msgs::Bool::ConstPtr &msg);
  void GoDisinfectionStationCallback(const std_msgs::Bool::ConstPtr &msg);
  void executeCB(const action::TaskGoalConstPtr &execute_goal);
  void newGoal(const action::TaskGoalConstPtr &goal);
  void preemptCB();

protected:
  ros::NodeHandle node;
  ros::NodeHandle load_site;  //使用站点命名空间加载参数
  ros::Time end_time;
  ros::Duration work_time;
  ros::Publisher pub_gohome;
  ros::Subscriber sub_ESP32,sub_go_home,sub_go_disinfection_station;
  ros::ServiceClient client_work,client_clear_costmaps,client_clear_unknown_space;
  move_base_msgs::MoveBaseGoal Goal[NUM],Goal_home,Goal_disinfection_station;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_Movebase;
  actionlib::SimpleActionServer<action::TaskAction> as_Task;
  action::TaskResult resul_;
  action::TaskFeedback feedback_;
  srv::work work_;
  std_srvs::Empty clear_costmaps_,clear_unknown_space_;
  std_msgs::Bool gohome;
  string action_name_,route_name;    //action名称,路线名称
  uint8_t work_mode,disinfect_mode;     //巡航工作模式,//路线点位数量,//消毒模式
  int Target_Point_Number;
  bool isSufficientDisinfectant,isSufficientPower,is_go_home,is_go_disinfectant;    //消毒液是否充足, //电量是否充足

};

//**********任务action目标处理回调函数*************//
void TaskActionServer::newGoal(const action::TaskGoalConstPtr &goal)
{
  bool tasking=true;
  //任务要求
  route_name=goal->RouteName;
  work_mode=goal->WorkMode;
  disinfect_mode=goal->DisinfectMode;
  work_time=ros::Duration(60*goal->WorkTime);
  loop_:

  //加载目标点位信息
  for (int i=1;i<=Target_Point_Number;i++) {
    Goal[i].target_pose.header.frame_id="map";
    node.getParam(route_name+"/positionX_"+to_string(i),Goal[i].target_pose.pose.position.x);
    node.getParam(route_name+"/positionY_"+to_string(i),Goal[i].target_pose.pose.position.y);
    node.getParam(route_name+"/positionZ_"+to_string(i),Goal[i].target_pose.pose.position.z);
    node.getParam(route_name+"/orientationX_"+to_string(i),Goal[i].target_pose.pose.orientation.x);
    node.getParam(route_name+"/orientationY_"+to_string(i),Goal[i].target_pose.pose.orientation.y);
    node.getParam(route_name+"/orientationZ_"+to_string(i),Goal[i].target_pose.pose.orientation.z);
    node.getParam(route_name+"/orientationW_"+to_string(i),Goal[i].target_pose.pose.orientation.w);
  }
  end_time=ros::Time::now()+work_time;
  //机器人运动顺序标志位 isOder=true 顺序    isOder=false 顺序
  bool isOder=true;
  uint8_t clear_costmap_num = 0;  //清除地图障碍物次数
  for (int i=1;i<=Target_Point_Number && ros::ok();)
  {
    //等待move_base服务端开启
    ac_Movebase.waitForServer();
    cout<<"move_base action server start"<<endl;
    Goal[i].target_pose.header.stamp=ros::Time::now();
    //发布导航目标点
    ac_Movebase.sendGoal(Goal[i]);


    //在前往目标点的路上，以10秒一次的频率检查任务时间、电量和液位是否充足,用户是否取消任务
    ros::Rate loop_rate(0.1);
    while(ros::ok())
    {
      //判断任务要求时间是否到达
      if(ros::Time::now()>=end_time)
      {
        cout<<"任务时间到了，结束任务"<<endl;
        //取消当前目标
        ac_Movebase.cancelGoal();
        resul_.Resul=resul_successful;
        as_Task.setSucceeded(resul_); //返回任务成功
        gohome.data=true;
        //呼叫GoHome话题
        pub_gohome.publish(gohome);
        tasking=false;
        break;
      }

      //更新新任务
      if(as_Task.isPreemptRequested()) {
        if(as_Task.isNewGoalAvailable()) {
          cout<<"接受新任务"<<endl;
          action::TaskGoal new_goal=*as_Task.acceptNewGoal();
          //取消当前导航目标点
          ac_Movebase.cancelGoal();
          route_name=new_goal.RouteName;
          work_mode=new_goal.WorkMode;
          disinfect_mode=new_goal.DisinfectMode;
          work_time=ros::Duration(60*new_goal.WorkTime);//接收new任务
          goto loop_;
        }
        else {
          //提前结束任务
        }
      }
      // 确保任务还没有被取消
      if(!as_Task.isActive()||!ros::ok()) {
        tasking=false;
        break;
      }
      //反馈机器人正在前往目标点
      feedback_.Feedback=feedback_go;
      as_Task.publishFeedback(feedback_);

      //电量不足，停止工作,前往充电站
      if(isSufficientPower==false)
      {
        cout<<"电源不足,停止工作，取消目标"<<endl;
        work_.request.working_start=false;
        client_work.call(work_);
        resul_.Resul=resul_Insufficient_power;      //返回电量不足
        as_Task.setAborted(resul_);     //设置行为崩溃
        //取消当前目标
        ac_Movebase.cancelGoal();
        gohome.data=true;
        //呼叫GoHome话题
        pub_gohome.publish(gohome);
        tasking=false;
        break;
      }

      //消毒液不足，停止工作,前往换液站
      if(isSufficientDisinfectant==false) {
         //向客户端反馈消毒液不足
         feedback_.Feedback=feedback_Insufficient_disinfectant;
         as_Task.publishFeedback(feedback_);
         //停止消毒任务
         work_.request.working_start=false;
         client_work.call(work_);
         cout<<"消毒液不足，停止工作,前往换液站"<<endl;
         //取消move_base导航目标点位
         ac_Movebase.cancelGoal();
         Goal_disinfection_station.target_pose.header.stamp=ros::Time::now();
         //发布move_base导航到加液站点位
         ac_Movebase.sendGoal(Goal_disinfection_station);
         ac_Movebase.waitForResult();
         if(ac_Movebase.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
           cout<<"到达换液站"<<endl;
           ros::Rate loop(5);
           while(ros::ok()) {
             if(isSufficientDisinfectant==true) {
               cout<<"更换消毒液成功"<<endl;
               feedback_.Feedback=feedback_Disinfectant_replaced_successfully;   //反馈消毒液更换成功
               as_Task.publishFeedback(feedback_);
               sleep(3);     //延时3秒
               break;
             }
             else {
               cout<<"等待更换消毒液！"<<endl;
             }
             loop.sleep();
             ros::spinOnce();
           }
           //继续前往更换消毒液前的路线下一点
           if(isOder==true)
             i--;
           else
             i++;
           break;
         }
         else {
           cout<<"暂时无法到达加液站，稍后将自动重试"<<endl;
           if(isOder==true)
             i--;
           else
             i++;
           break;
         }
      }

      //若计划流产，呼叫clear_costmaps服务
      if(ac_Movebase.getState() != actionlib::SimpleClientGoalState::SUCCEEDED && clear_costmap_num <= 2) {
        ROS_ERROR("move_base is not SUCCEEDED");
        cout<<"未到达路线"<<route_name<<"上的目标点"<<i<<"即将清除在代价地图中的障碍"<<endl;
        client_clear_costmaps.call(clear_costmaps_);
        clear_costmap_num++;
        printf("clear_costmap_num=%d\n",clear_costmap_num);
        sleep(3); //延时3s
        //当清除代价地图障碍次数不小于2时，机器人放弃目标点，前往下一目标点
        if(clear_costmap_num < 2) {
          if(isOder==true)
            i--;
          else
            i++;
          continue;
        }
        else {
          break;
        }
      }
      //等待move_base结果返回
      ac_Movebase.waitForResult();
      if (ac_Movebase.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        clear_costmap_num=0;//清理代价地图次数归零
        //成功达到目标点i
        cout<<"到达路线"<<route_name<<"上的目标点"<<i<<endl;

        //工作模式为巡航消毒
        if(work_mode==0) {
          if(work_.response.working_feedback==false) {
            work_.request.working_way=disinfect_mode;
            work_.request.working_start=true;
            client_work.call(work_);
            //当到达目标点前未开启工作模式，延时10秒
            sleep(10);
            break;
          }
          else {
            //到达目标点后延时5秒，原地停止工作
            feedback_.Feedback=feedback_Disinfecting;
            as_Task.publishFeedback(feedback_);
            sleep(5);
            break;
          }
        }
        //工作模式为定点消毒
        else {
          if(work_.response.working_feedback==false) {
            work_.request.working_way=disinfect_mode;
            work_.request.working_start=true;
            client_work.call(work_);
            feedback_.Feedback=feedback_Disinfecting;
            as_Task.publishFeedback(feedback_);
            //当到达目标点前未开启工作模式，延时10秒
            sleep(15);
            work_.request.working_way=3;
            work_.request.working_start=false;
            client_work.call(work_);
            //在点位喷雾后关闭喷雾
            sleep(5);
            break;
          }
        }
      }
      loop_rate.sleep();
      ros::spinOnce();
    }

    //在首尾点位判断顺序标志位
    if(i==Target_Point_Number && isOder==true)
      isOder=false;
    if(i==1 && isOder==false)
      isOder=true;

    //点位顺序处理
    if(isOder==true)
      i++;
    else
      i--;

    if(tasking==false)
      break;
  }
}

////**********任务action反馈处理回调函数*************//
void TaskActionServer::preemptCB()
{
  cout<<"中断当前任务"<<route_name<<endl;
  resul_.Resul=resul_cancel;
  as_Task.setPreempted(resul_);  //强制中断
  ac_Movebase.cancelGoal();
}

//*********消毒控制版ESP32数据处理回调函数***** ****//
void TaskActionServer::ESP32Callback(const msg::esp32_disinfect::ConstPtr &msg)
{
  if(msg->esp32_disinfect_data[2]<=15) {
    //消毒液不足
    isSufficientDisinfectant=false;
  }
  if(msg->esp32_disinfect_data[2]>=95) {
    //消毒液充足
    isSufficientDisinfectant=true;
  }
  if(msg->esp32_disinfect_data[3]<=20) {
    //电量不足
    isSufficientPower=false;
  }
}

////*********GoHome回调函数***********//
void TaskActionServer::GoHomeCallback(const std_msgs::Bool::ConstPtr &msg)
{
  is_go_home=msg->data;
  //确保发布回充电桩指令
  if(is_go_home==false)
    return;
  //如果Task action server是活跃的，设置放弃
  if(as_Task.isActive())
    as_Task.setAborted();
  //请求取消正在动作服务器上运行的所有目标:
  ac_Movebase.cancelAllGoals();
  usleep(1000*100);

  cout<<"前往充电站"<<endl;
  Goal_home.target_pose.header.stamp=ros::Time::now();
  ac_Movebase.sendGoal(Goal_home);
  ac_Movebase.waitForResult();
  //10s一次的频率判断是否到达充电站
  ros::Rate loop_rate(0.1);
  while(ros::ok()) {
    if(ac_Movebase.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      cout<<"到达充电站"<<endl;
      break;
    }
    else {
      cout<<"暂时无法到达充电桩，稍后重试"<<endl;
      Goal_home.target_pose.header.stamp=ros::Time::now();
      ac_Movebase.sendGoal(Goal_home);
    }
    loop_rate.sleep();
    ros::spinOnce();
  }
}

//*********GoDisinfectant回调函数***********//
void TaskActionServer::GoDisinfectionStationCallback(const std_msgs::Bool::ConstPtr &msg)
{
  is_go_disinfectant=msg->data;
  if(is_go_disinfectant==false)
    return;
  //如果Task action server是活跃的，设置放弃
  if(as_Task.isActive())
    as_Task.setAborted();
  //请求取消正在动作服务器上运行的所有目标:
  ac_Movebase.cancelAllGoals();
  usleep(1000*100);

  Goal_disinfection_station.target_pose.header.stamp=ros::Time::now();
  ac_Movebase.sendGoal(Goal_disinfection_station);
  ac_Movebase.waitForResult();
  //10s一次的频率判断是否到达充电站
  ros::Rate loop_rate(0.1);
  while(ros::ok()) {
    if(ac_Movebase.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      cout<<"到达加液站"<<endl;
      break;
    }
    else {
      cout<<"暂时无法到达加液站，稍后重试"<<endl;
      Goal_disinfection_station.target_pose.header.stamp=ros::Time::now();
      ac_Movebase.sendGoal(Goal_disinfection_station);
    }
    loop_rate.sleep();
    ros::spinOnce();
  }
}

//**********析构函数****************//
TaskActionServer::~TaskActionServer()
{
  if(work_.response.working_feedback==true)
  {
    work_.request.working_start=false;
    client_work.call(work_);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"sr_task_action_server");
  TaskActionServer Task("sr_task_action_server");
  ros::spin();
  return 0;
}