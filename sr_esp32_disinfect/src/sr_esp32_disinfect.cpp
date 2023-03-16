#include"ros/ros.h"
#include "serial/serial.h"
#include"msg/esp32_disinfect.h"
#include"srv/work.h"
using namespace std;
uint8_t buffer[10];
ros::Publisher pub_esp32;
ros::ServiceServer ser_work;
serial::Serial sp; 
static uint8_t buf[4];
bool turtlecontrolback(srv::work::Request &req,srv::work::Response &res)
{
  if(req.working_start)
  {
      buf[0]=0xFF;
      buf[1]=0x00;
      buf[2]=0x00;
      buf[3]=0x11;
  }
  else
  {
    buf[0]=0xFF;
    buf[1]=0x00;
    buf[2]=0x00;
    buf[3]=0x00;
  }
  sp.write(buf,4);
  res.working_feedback=true;
}
void translate_mb_data(void)
{
 msg::esp32_disinfect  msg;
 for (int i=0;i<4;i++)
 {
  msg.esp32_disinfect_data[i]=buffer[i];
 }
pub_esp32.publish(msg);
}
int  main(int argc, char **argv)
{
  ros::init(argc,argv,"sr_esp32_disinfect_node");
  ros::NodeHandle n;
  string Ports;
  n.getParam("~Ports",Ports);
  pub_esp32=n.advertise<msg::esp32_disinfect>("esp32_disinfect_data",1000);
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
      ROS_INFO_STREAM("sr_other_sensors is opened.");
  }
  else
  {
      return -1;
  }
  ros::Rate loop_rate(20);
  sp.flushInput();

  ser_work=n.advertiseService("work",turtlecontrolback);

  while (ros::ok())
  {

  int n = sp.read(buffer,10);
  if(n==4)
  {
    translate_mb_data();
  }
  loop_rate.sleep();
  ros::spinOnce();
  }

  return 0;
}
