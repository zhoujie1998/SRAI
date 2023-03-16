/**********************************
 **********串口屏幕ROS驱动***********
 *********************************/
#include"ros/ros.h"
#include"serial/serial.h"
#include"msg/esp32_disinfect.h"
#include"geometry_msgs/Twist.h"
#include"srv/work.h"
#include"std_msgs/Bool.h"

serial::Serial sp;
ros::ServiceServer ser_work;
//标志位
//flag_disinfecting :是否正在消毒
//flag_moving       :是否正在移动
//flag_pausing      :是否急停中
int flag=-1,flag_pausing=0,flag_moving=0,flag_disinfecting=0,level_flag=4,power_flag=3,power_H=0,power_L=0x64,charge=0;
void handle_flag()
{
    if(flag_disinfecting==0&&flag_moving==0&&flag_pausing==0)
  {
    flag=-1;  //加载图片：开机中
  }
  else if (flag_disinfecting==1&&flag_pausing==0)  //正在消毒，并且没有急停
  {
    flag=0;   //加载图片：正在消毒
  }
  else if (flag_disinfecting==0&&flag_moving==1&&flag_pausing==0) //轮子有速度反馈
  {
    flag=1;   //加载图片：正在导航
  }
  else if (flag==-1&&flag_moving==1&&flag_pausing==0)
  {
    flag=1;   //
  }
 else if(flag_pausing==1)  //急停开关判断
  {
    flag=4;   //加载图片：急停中
  }
}
//消毒工作服务是否开启
bool turtlecontrolback(srv::work::Request &req,srv::work::Response &res)
{

  if(req.working_start)
  {
   flag_disinfecting=1;
  }
  else
  {
   flag_disinfecting=0;
  }

}
//轮子速度反馈回调函数
void feedback_cmd_vel(const geometry_msgs::Twist::ConstPtr&msg)
{

  if((msg->linear.x!=0)||(msg->angular.z!=0))
  {
    flag_moving=1;
  }
  else
  {
    flag_moving=0;
  }

}

//急停回调函数
void feedback_stop(const std_msgs::Bool::ConstPtr&msg)
{
  if(msg->data==true)   //注意可能需要更改
  {
    flag_pausing=1;
  }
  else
  {
    flag_pausing=0;
  }
}
//ESP32控制板上报的液位信息及电量
void feedback_display(const msg::esp32_disinfect::ConstPtr&msg)
{
  for (int i=0;i<5;i++)
  {
    if(20*i+20>msg->esp32_disinfect_data[2]&&msg->esp32_disinfect_data[2]>20*i)
    {
      level_flag=i;
      break;
    }
    else if (msg->esp32_disinfect_data[2]==20*(i+1))
    {
      level_flag=i+1;
      break;
    }
 if (msg->esp32_disinfect_data[2]==0)
    {
      level_flag=-1;
      break;
    }
  }
  for (int i=0;i<4;i++)
  {
    if((i+1)*25>=msg->esp32_disinfect_data[3]&&msg->esp32_disinfect_data[3]>25*i)
    {
      power_flag=i;
      break;
    }
    else if(msg->esp32_disinfect_data[3]>100)
    {
      power_flag=3;
    }
    else if(msg->esp32_disinfect_data[3]==0||msg->esp32_disinfect_data[3]<0)
    {
      power_flag=0;
    }
  }
if(msg->esp32_disinfect_data[0]==0xFF)
{
  charge=1;
}
else
{
  charge=0;
}
  power_H=(msg->esp32_disinfect_data[3]&0xFF00)>>8;
  power_L=msg->esp32_disinfect_data[3]&0x00FF;
}

int  main(int argc, char **argv)
{
  ros::init(argc,argv,"sr_screen");
  ros::NodeHandle n;
  std::string Ports;
  n.getParam("sr_screen_node/Ports",Ports);
  sp.setPort(Ports);
  serial::Timeout to = serial::Timeout::simpleTimeout(100);
  sp.setBaudrate(115200);
  sp.setTimeout(to);
  try
  {
      sp.open();
  }
  catch(serial::IOException& e)
  {
      ROS_ERROR_STREAM("Unable to open sr_screen.");
      return -1;
  }
  if(sp.isOpen())
  {
      ROS_INFO_STREAM("sr_screen is opened.");
  }
  else
  {
      return -1;
  }

sp.flushInput();

ros::Subscriber sub_screen_workdata,sub_screen_displaydata,sub_screen_stop;
sub_screen_workdata= n.subscribe("/feedback/cmd_vel",1000,feedback_cmd_vel);
sub_screen_displaydata= n.subscribe("esp32_disinfect_data",1000,feedback_display);
sub_screen_stop= n.subscribe("/stop",1000,feedback_stop);
ser_work=n.advertiseService("work",turtlecontrolback);
uint8_t screen_cmd[8],screen_dispay[10],screen_level[8],screen_power[8],screen_powerquantity[8],screen_charge[8];
while (ros::ok())
{
  handle_flag();
  switch(flag)
  {
  case -1:
    screen_cmd[0]=0x5A;
    screen_cmd[1]=0xA5;
    screen_cmd[2]=0x05;
    screen_cmd[3]=0x82;
    screen_cmd[4]=0x20;
    screen_cmd[5]=0x00;
    screen_cmd[6]=0x00;
    screen_cmd[7]=0x01;
    break;
  case 0:
    screen_cmd[0]=0x5A;
    screen_cmd[1]=0xA5;
    screen_cmd[2]=0x05;
    screen_cmd[3]=0x82;
    screen_cmd[4]=0x20;
    screen_cmd[5]=0x00;
    screen_cmd[6]=0x00;
    screen_cmd[7]=0x02;
    break;
  case 1:
    screen_cmd[0]=0x5A;
    screen_cmd[1]=0xA5;
    screen_cmd[2]=0x05;
    screen_cmd[3]=0x82;
    screen_cmd[4]=0x20;
    screen_cmd[5]=0x00;
    screen_cmd[6]=0x00;
    screen_cmd[7]=0x00;
    break;
  case 4:
    screen_cmd[0]=0x5A;
    screen_cmd[1]=0xA5;
    screen_cmd[2]=0x05;
    screen_cmd[3]=0x82;
    screen_cmd[4]=0x20;
    screen_cmd[5]=0x00;
    screen_cmd[6]=0x00;
    screen_cmd[7]=0x04;
    break;
  }
  switch (level_flag)
  {
  case -1:
    screen_dispay[0]=0x5A;
    screen_dispay[1]=0xA5;
    screen_dispay[2]=0x07;
    screen_dispay[3]=0x82;
    screen_dispay[4]=0x00;
    screen_dispay[5]=0x84;
    screen_dispay[6]=0x5A;
    screen_dispay[7]=0x01;
    screen_dispay[8]=0x00;
    screen_dispay[9]=0x01;

    screen_level[0]=0x5A;
    screen_level[1]=0xA5;
    screen_level[2]=0x05;
    screen_level[3]=0x82;
    screen_level[4]=0x21;
    screen_level[5]=0x00;
    screen_level[6]=0x00;
    screen_level[7]=0x00;
    break;
  case 0:
    screen_dispay[0]=0x5A;
    screen_dispay[1]=0xA5;
    screen_dispay[2]=0x07;
    screen_dispay[3]=0x82;
    screen_dispay[4]=0x00;
    screen_dispay[5]=0x84;
    screen_dispay[6]=0x5A;
    screen_dispay[7]=0x01;
    screen_dispay[8]=0x00;
    screen_dispay[9]=0x00;

    screen_level[0]=0x5A;
    screen_level[1]=0xA5;
    screen_level[2]=0x05;
    screen_level[3]=0x82;
    screen_level[4]=0x21;
    screen_level[5]=0x00;
    screen_level[6]=0x00;
    screen_level[7]=0x00;
    break;
   case 1:
    screen_dispay[0]=0x5A;
    screen_dispay[1]=0xA5;
    screen_dispay[2]=0x07;
    screen_dispay[3]=0x82;
    screen_dispay[4]=0x00;
    screen_dispay[5]=0x84;
    screen_dispay[6]=0x5A;
    screen_dispay[7]=0x01;
    screen_dispay[8]=0x00;
    screen_dispay[9]=0x00;

    screen_level[0]=0x5A;
    screen_level[1]=0xA5;
    screen_level[2]=0x05;
    screen_level[3]=0x82;
    screen_level[4]=0x21;
    screen_level[5]=0x00;
    screen_level[6]=0x00;
    screen_level[7]=0x01;
    break;
   case 2:
    screen_dispay[0]=0x5A;
    screen_dispay[1]=0xA5;
    screen_dispay[2]=0x07;
    screen_dispay[3]=0x82;
    screen_dispay[4]=0x00;
    screen_dispay[5]=0x84;
    screen_dispay[6]=0x5A;
    screen_dispay[7]=0x01;
    screen_dispay[8]=0x00;
    screen_dispay[9]=0x00;

    screen_level[0]=0x5A;
    screen_level[1]=0xA5;
    screen_level[2]=0x05;
    screen_level[3]=0x82;
    screen_level[4]=0x21;
    screen_level[5]=0x00;
    screen_level[6]=0x00;
    screen_level[7]=0x02;
    break;
  case 3:
    screen_dispay[0]=0x5A;
    screen_dispay[1]=0xA5;
    screen_dispay[2]=0x07;
    screen_dispay[3]=0x82;
    screen_dispay[4]=0x00;
    screen_dispay[5]=0x84;
    screen_dispay[6]=0x5A;
    screen_dispay[7]=0x01;
    screen_dispay[8]=0x00;
    screen_dispay[9]=0x00;

    screen_level[0]=0x5A;
    screen_level[1]=0xA5;
    screen_level[2]=0x05;
    screen_level[3]=0x82;
    screen_level[4]=0x21;
    screen_level[5]=0x00;
    screen_level[6]=0x00;
    screen_level[7]=0x03;
   break;
  case 4:
    screen_dispay[0]=0x5A;
    screen_dispay[1]=0xA5;
    screen_dispay[2]=0x07;
    screen_dispay[3]=0x82;
    screen_dispay[4]=0x00;
    screen_dispay[5]=0x84;
    screen_dispay[6]=0x5A;
    screen_dispay[7]=0x01;
    screen_dispay[8]=0x00;
    screen_dispay[9]=0x00;

    screen_level[0]=0x5A;
    screen_level[1]=0xA5;
    screen_level[2]=0x05;
    screen_level[3]=0x82;
    screen_level[4]=0x21;
    screen_level[5]=0x00;
    screen_level[6]=0x00;
    screen_level[7]=0x04;
   break;
  case 5:
    screen_dispay[0]=0x5A;
    screen_dispay[1]=0xA5;
    screen_dispay[2]=0x07;
    screen_dispay[3]=0x82;
    screen_dispay[4]=0x00;
    screen_dispay[5]=0x84;
    screen_dispay[6]=0x5A;
    screen_dispay[7]=0x01;
    screen_dispay[8]=0x00;
    screen_dispay[9]=0x00;

    screen_level[0]=0x5A;
    screen_level[1]=0xA5;
    screen_level[2]=0x05;
    screen_level[3]=0x82;
    screen_level[4]=0x21;
    screen_level[5]=0x00;
    screen_level[6]=0x00;
    screen_level[7]=0x04;
   break;
  }
  screen_powerquantity[0]=0x5A;
  screen_powerquantity[1]=0xA5;
  screen_powerquantity[2]=0x05;
  screen_powerquantity[3]=0x82;
  screen_powerquantity[4]=0x24;
  screen_powerquantity[5]=0x00;
  screen_powerquantity[6]=power_H;
  screen_powerquantity[7]=power_L;
  switch (power_flag)
  {
  case 0:
    screen_power[0]=0x5A;
    screen_power[1]=0xA5;
    screen_power[2]=0x05;
    screen_power[3]=0x82;
    screen_power[4]=0x23;
    screen_power[5]=0x00;
    screen_power[6]=0x00;
    screen_power[7]=0x00;
    break;
  case 1:
    screen_power[0]=0x5A;
    screen_power[1]=0xA5;
    screen_power[2]=0x05;
    screen_power[3]=0x82;
    screen_power[4]=0x23;
    screen_power[5]=0x00;
    screen_power[6]=0x00;
    screen_power[7]=0x01;
    break;
  case 2:
    screen_power[0]=0x5A;
    screen_power[1]=0xA5;
    screen_power[2]=0x05;
    screen_power[3]=0x82;
    screen_power[4]=0x23;
    screen_power[5]=0x00;
    screen_power[6]=0x00;
    screen_power[7]=0x02;
    break;
  case 3:
    screen_power[0]=0x5A;
    screen_power[1]=0xA5;
    screen_power[2]=0x05;
    screen_power[3]=0x82;
    screen_power[4]=0x23;
    screen_power[5]=0x00;
    screen_power[6]=0x00;
    screen_power[7]=0x03;
    break;
  }
  switch(charge)
  {
  case 0:
    screen_charge[0]=0x5A;
    screen_charge[1]=0xA5;
    screen_charge[2]=0x05;
    screen_charge[3]=0x82;
    screen_charge[4]=0x22;
    screen_charge[5]=0x00;
    screen_charge[6]=0x00;
    screen_charge[7]=0x00;
    break;
  case 1:
    screen_charge[0]=0x5A;
    screen_charge[1]=0xA5;
    screen_charge[2]=0x05;
    screen_charge[3]=0x82;
    screen_charge[4]=0x22;
    screen_charge[5]=0x00;
    screen_charge[6]=0x00;
    screen_charge[7]=0x01;
    break;
  }
 sp.write(screen_cmd,8);
 sp.write(screen_level,8);
 sp.write(screen_power,8);
 sp.write(screen_charge,8);
 sp.write(screen_dispay,10);
 sp.write(screen_powerquantity,8);

 ros::spinOnce();
}
}
