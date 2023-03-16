/*
通过自定义action Task 得到任务路线名，工作模式，消毒模式，工作时间
再通过传目标点给move_base
根据消毒控制板ESP32上报的液位和电池上报的电量数据来自动发布相应目标站点服务，
实现在任务中液位不足可自动前往加液站，加液、充电完成后继续任务

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
#include "sr_navigation.h"

/*************************************************
 函数名称：GetYamlParam
 函数功能：获取指定参数文件中的点位数据
 函数参数：filename: 文件名
 函数返回：是否成功读取指定参数文件
**************************************************/
bool TaskActionServer::GetYamlParam(const std::string &filename)
{
  string file = package_dir+"/param/"+filename+".yaml";
  YAML::Node config;
  //尝试打开指定路线文件
  try{
    config = YAML::LoadFile(file);
    Target_Point_Number = config[filename]["Target_Point_Number"].as<int>();
    // voctor扩容，满足路线点位存储内存
    Goal_points.resize(Target_Point_Number);
    for (int i=0 ; i<Target_Point_Number ; i++) {
    Goal_points[i].target_pose.header.frame_id = "map";
    Goal_points[i].target_pose.pose.position.x = config[filename]["positionX_"+to_string(i)].as<float>();
    Goal_points[i].target_pose.pose.position.y = config[filename]["positionY_"+to_string(i)].as<float>();
    Goal_points[i].target_pose.pose.position.z = config[filename]["positionZ_"+to_string(i)].as<float>();
    Goal_points[i].target_pose.pose.orientation.x = config[filename]["orientationX_"+to_string(i)].as<float>();
    Goal_points[i].target_pose.pose.orientation.y = config[filename]["orientationY_"+to_string(i)].as<float>();
    Goal_points[i].target_pose.pose.orientation.z = config[filename]["orientationZ_"+to_string(i)].as<float>();
    Goal_points[i].target_pose.pose.orientation.w = config[filename]["orientationW_"+to_string(i)].as<float>();
    }
  } 
  catch(const YAML::Exception& e) {
    ROS_ERROR_STREAM( e.what());
    ROS_ERROR_STREAM("read "<<filename<<".yaml error!");
    return false;
  }
  return true;
}

/*************************************************
 函数名称：CheckState
 函数功能：检查任务、时间、电量、液位的状态
 函数参数：无
 函数返回：任务是否继续
**************************************************/
bool TaskActionServer::CheckState()
{
  //更新新任务
  if(as_Task.isPreemptRequested()) {
    if(as_Task.isNewGoalAvailable()) {
      ROS_WARN("Accept new tasks");
      action::TaskGoalConstPtr  new_goal = as_Task.acceptNewGoal(); 
      //取消当前导航目标点
      ac_Movebase.cancelGoal();
      //调用任务回调函数 传入新任务
      NewGoal(new_goal,0);
    }
    else {
      ROS_INFO_STREAM("End task ahead of schedule");
    }
  }

  // 判断任务是否处于活动状态
  if(!as_Task.isActive()) {
    ROS_WARN_STREAM("Task is not active!");
    return false;
  }

  //判断任务时间是否到达
  if( ros::Time::now() >= end_time )
  {
    ROS_WARN_STREAM("Task time is up, end of the task");
    //取消当前目标
    ac_Movebase.cancelGoal();
    resul_.Resul = resul_successful;
    //返回任务成功
    as_Task.setSucceeded(resul_);  
    gosite_.request.SiteName = home_name;
    //GoSite服务
    GoSiteCallback(gosite_.request,gosite_.response);
    return false;
  }

  //巡航途中(或在等待更换消毒液时)，电量不足，停止工作，前往充电站
  if( (goto_way == true && SufficientPower == false) || (SufficientPower == false && SufficientDisinfectant == false)) {
    ROS_WARN_STREAM("Insufficient power, stop working");
    resul_.Resul = resul_Insufficient_power;  //返回电量不足
    work_.request.working_start = false;
    client_work.call(work_);
    ac_Movebase.cancelGoal();
    //GoSite服务
    gosite_.request.SiteName = charge_name;
    GoSiteCallback(gosite_.request,gosite_.response);
  }
 
  //巡航途中，消毒液不足，停止工作，前往换液站
  if( goto_way == true && SufficientDisinfectant == false) {
      ROS_WARN_STREAM("Stop work, because the disinfectant is insufficient, and go to the disinfectant station");
      //向客户端反馈消毒液不足
      feedback_.Feedback=feedback_Insufficient_disinfectant;
      as_Task.publishFeedback(feedback_);
      //停止消毒任务
      work_.request.working_start=false;
      client_work.call(work_);
      //取消move_base导航目标点位
      ac_Movebase.cancelGoal();
      //GoSite服务
      gosite_.request.SiteName = disinfection_name;
      GoSiteCallback(gosite_.request,gosite_.response);
  }

   //未在巡航途中，电量、消毒液充足，回到航线
  if(SufficientPower == true && SufficientDisinfectant == true && goto_way == false) {
    GoalPtr->WorkMode = GoalConstPtr->WorkMode;
    GoalPtr->DisinfectMode = GoalConstPtr->DisinfectMode;
    GoalPtr->RouteName = GoalConstPtr->RouteName;
    GoalPtr->WorkTime = (ros::Duration (end_time-ros::Time::now()).toSec())/60.0;
    ROS_WARN("WorkTime have %d minute.",GoalPtr->WorkTime);
    GoalConstPtr = GoalPtr;
    NewGoal(GoalConstPtr,goal_point);
  }
  return true;
}

/*************************************************
 函数名称：NewGoal
 函数功能：根据任务Task导航、消毒工作
 函数参数：goal:动作目标内容，count:巡航起始点,默认为0
 函数返回：无
**************************************************/
void TaskActionServer::NewGoal(const action::TaskGoalConstPtr &goal,int count_ )
{
  goto_way = true;
  end_time = ros::Time::now()+ros::Duration(60*goal->WorkTime);
  GoalConstPtr = goal;
  //任务要求
  ROS_INFO_STREAM("Task Action CallBack");
  
  //加载目标点位信息,若参数文件无指定路线，返回false
  if(!GetYamlParam(goal->RouteName)) {
    //返回路线名称错误
    resul_.Resul=resul_way_file_error;      
    return;
  }
  //机器人运动顺序标志位 isOder: true 顺序    isOder: false 顺序
  bool isOder=true;
  //点位尝试次数初始化
  uint8_t tryNumber = 1; 
  ROS_INFO_STREAM("---------------------------------------------------------------");
  for (uint8_t i = count_;i<Target_Point_Number;) {
    goal_point = i;
    //等待move_base服务端开启
    ac_Movebase.waitForServer();
    ROS_INFO("GO the way: \033[1;32m%s\033[0m,print->\033[1;32m %d\033[0m",goal->RouteName.c_str(),i);
    Goal_points[i].target_pose.header.stamp=ros::Time::now();
    //发布导航目标点
    ac_Movebase.sendGoal(Goal_points[i]);
    //以2.5秒一次的频率等待move_base结果返回，并监听任务
    while(ros::ok())  {
      ros::spinOnce();
      if(!CheckState()) {
        return;
      }
      if(  goto_way == false) {
        if(SufficientPower == false)
        ROS_INFO("Charging");
        if(SufficientDisinfectant == false)
        ROS_INFO("Add disinfectant");
        sleep(3);
        continue;
      }
      //反馈机器人正在前往目标点
      feedback_.Feedback=feedback_go;
      as_Task.publishFeedback(feedback_);
      if(ac_Movebase.waitForResult(ros::Duration(3.0))) break;
    }
    //若计划流产，呼叫clear_costmaps服务，并且再次尝试(允许尝试2次)
    if(ac_Movebase.getState() == actionlib::SimpleClientGoalState::ABORTED && tryNumber < 2) {
      client_clear_costmaps.call(clear_);
      tryNumber++;
      ROS_ERROR_STREAM("\033[1;31m--------------------------\033[0m");
      continue;
    }

    if (ac_Movebase.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) {
      //点位尝试次数初始化
      tryNumber = 1; 
      ROS_INFO("\033[1;32mReached the way: %s,print-> %d is SUCCEEDED\033[0m",goal->RouteName.c_str(),i);
      //工作模式为巡航消毒
      if(goal->WorkMode == 0) {
        ROS_INFO_STREAM("wait,Disinfecting now");
        if(work_.response.working_feedback == false) {
          work_.request.working_way = goal->DisinfectMode;
          //开启消毒系统
          work_.request.working_start = true;
          client_work.call(work_);
          //当到达目标点前未开启工作模式，延时10秒
          ROS_INFO_STREAM("Wait 10 seconds");
          sleep(10);
          feedback_.Feedback=feedback_Disinfecting;
          as_Task.publishFeedback(feedback_);
        }
        else {
          //到达目标点后延时5秒，原地喷雾
          feedback_.Feedback = feedback_Disinfecting;
          as_Task.publishFeedback(feedback_);
          ROS_INFO_STREAM("Wait 5 seconds");
          sleep(5);
        }
      }
      //工作模式为定点消毒
      else {
        ROS_INFO_STREAM("wait,Disinfecting now");
        if(work_.response.working_feedback==false) {
          work_.request.working_way = goal->DisinfectMode;
          //开启消毒系统
          work_.request.working_start = true;
          client_work.call(work_);
          feedback_.Feedback = feedback_Disinfecting;
          as_Task.publishFeedback(feedback_);
          ROS_INFO_STREAM("Wait 15 seconds");
          sleep(15);
          //关闭消毒系统
          work_.request.working_start = false;
          client_work.call(work_);
          sleep(5);
        }
      }
    }
    
    //在首尾点位判断顺序标志位
    if(i >= Target_Point_Number-1 && isOder == true) {
      isOder = false;
      client_clear_costmaps.call(clear_);
      sleep(5);
      //client_clear_unknown_space.call(clear_);
    }

    if(i <= 0 && isOder == false) {
      isOder = true;
      client_clear_costmaps.call(clear_);
      sleep(5);
      //client_clear_unknown_space.call(clear_);
    }

    if(tryNumber >= 2) ROS_ERROR("\033[1;31mGive up the point!\033[0m");
    //点位顺序处理    顺序和倒叙
    if(isOder == true)  i++;
    else i--;
    ROS_INFO_STREAM("---------------------------------------------------------------");
  }
}

/*************************************************
 函数名称：GoSite
 函数功能：前往站点服务端回调函数
 函数参数：req:服务目标，res:服务返回
 函数返回：服务是否完成
**************************************************/
bool TaskActionServer::GoSiteCallback(srv::goSite::Request &req,srv::goSite::Response &res)
{
  goto_way = false;
  //加载目标点位信息,若参数文件无指定路线，返回false
  if(!GetYamlParam(req.SiteName)) {
    res.Feedback = "Error loading point parameter file";      //加载点位参数文件错误
    return false;
  }
  //若目标站点是充电桩,重新配置参数,降低容差，提高导航回充精度
  //strstr() 模糊匹配
  if(strstr(req.SiteName.c_str(),"home") != NULL) {
    client.getCurrentConfiguration(dwa_config,ros::Duration(1));
    if(dwa_config.xy_goal_tolerance > 0.1 || dwa_config.yaw_goal_tolerance > 0.05) {
      dwa_config.xy_goal_tolerance = 0.1;
      dwa_config.yaw_goal_tolerance = 0.05;
      client.setConfiguration(dwa_config);
    }
  }
  //请求取消正在动作服务器上运行的所有目标:
  ac_Movebase.cancelAllGoals();
  usleep(1000*100);
  ROS_INFO_STREAM("---------------------------------------------------------------");
  ROS_INFO_STREAM("Go to the  site: "<<req.SiteName);
  Goal_points[0].target_pose.header.stamp = ros::Time::now();
  ac_Movebase.sendGoal(Goal_points[0]);
  ac_Movebase.waitForResult();
  //10s一次的频率判断是否到达站点
  ros::Rate loop_rate(0.1);
  while(ros::ok()) {
    if(ac_Movebase.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Reached the site: \033[1;32m%s \033[0mis SUCCEEDED",req.SiteName.c_str());
      ROS_INFO_STREAM("---------------------------------------------------------------");
      res.Feedback = "Successfully arrived at the site.";
      //恢复高容差
      client.getCurrentConfiguration(dwa_config,ros::Duration(1));
      if(dwa_config.xy_goal_tolerance < 0.2 || dwa_config.yaw_goal_tolerance < 0.1) {
        dwa_config.xy_goal_tolerance = 0.2;
        dwa_config.yaw_goal_tolerance = 0.1;
        client.setConfiguration(dwa_config);
      }
      return true;
    }
    else {
      res.Feedback = "Unable to reach the site, and will try again later";
      ROS_WARN_STREAM("Unable to reach the site: "<<req.SiteName<<", and will try again later");
      ROS_WARN_STREAM("\033[1;31m--------------------------\033[0m");
      Goal_points[0].target_pose.header.stamp = ros::Time::now();
      ac_Movebase.sendGoal(Goal_points[0]);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

//**********DWA动态调参回调函数*************//
void TaskActionServer::ConfigurationCallback(const dwa_local_planner::DWAPlannerConfig &config)
{
  ROS_INFO("a successful reconfiguration");
}

//**********任务action抢占回调函数*************//
void TaskActionServer::PreemptCallback()
{
  resul_.Resul = resul_cancel;
  //任务状态设置抢占
  as_Task.setPreempted(resul_);  
  ac_Movebase.cancelAllGoals();
}

//************电池电量回调函数************************//
void TaskActionServer::ChargeCallback(const msg::Battory::ConstPtr &msg)
{
  if(msg->Rest_Battory <= 15) {
    //电量不足
    SufficientPower = false;
  }
  if(msg->Rest_Battory >= 90) {
    //电量充足
    SufficientPower = true;
  }
}

//*********消毒控制版ESP32数据处理回调函数***** ****//
void TaskActionServer::ESP32Callback(const msg::esp32_disinfect::ConstPtr &msg)
{
  if(msg->esp32_disinfect_data[2] <= 15) {
    //消毒液不足
    SufficientDisinfectant = false;
  }
  if(msg->esp32_disinfect_data[2] >= 90) {
    //消毒液充足
    SufficientDisinfectant = true;
  }
}

//**********析构函数****************//
TaskActionServer::~TaskActionServer()
{
  if(work_.response.working_feedback == true)
  {
    //关闭消毒系统
    work_.request.working_start = false;
    client_work.call(work_);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"sr_task_action_server");
  int count = 0; //巡航起始点,默认为0
  TaskActionServer Task("sr_task_action_server",count);
  ros::spin();
  return 0;
}
