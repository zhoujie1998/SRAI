/*******************************************
************电应普 双目超声波A02 E08一拖四****************
********************************************/
#include "ros/ros.h"
#include "serial/serial.h"
#include "msg/ultrasound.h"
using namespace std;
uint8_t buffer[10];
ros::Publisher pub_ultrasound_data;
static uint8_t buf[8] = {0x01,0x06,0x02,0x02,0x00,0x01,0xE8,0x72};
void translate_mb_data(void)
{
 msg::ultrasound  msg;
 for (int i=0;i<4;i++)
 {
   msg.ultrasound_data[i]=buffer[2*i+1]*256+buffer[2*i+2];
 }
pub_ultrasound_data.publish(msg);
}

int  main(int argc, char **argv)
{
  ros::init(argc,argv,"binocular_ultrasound_node");
  ros::NodeHandle n;
  string Ports;
  ros::param::get("~Ports",Ports);
  pub_ultrasound_data=n.advertise<msg::ultrasound>("/ultrasound_data",100);
  serial::Serial sp;
  serial::Timeout to = serial::Timeout::simpleTimeout(100);
  sp.setPort(Ports);
  sp.setBaudrate(9600);
  sp.setTimeout(to);
  try
  {
      sp.open();
  }
  catch(serial::IOException& e)
  {
      ROS_ERROR_STREAM("Unable to open port.");
      return -1;
  }
  if(sp.isOpen())
  {
      ROS_INFO_STREAM("sr_anti_collision is opened.");
  }
  else
  {
      return -1;
  }
  //200ms循环一次
  ros::Rate loop_rate(5);
  sp.flushInput();

  sp.write(buf,8);
  while (ros::ok())
  {
    int n = sp.read(buffer,10);
    if(n==10)
     {
        translate_mb_data();
     }
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}


