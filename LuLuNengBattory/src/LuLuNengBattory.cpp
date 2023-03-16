#include<ros/ros.h>
#include<serial/serial.h>
#include<string>
#include<ros/publisher.h>
#include "msg/Battory.h"
#define baudrate (115200)
#define Length (21)
serial::Serial m_serial;
msg::Battory m_Battory;

std::string Ports;
//设在读取缓存
unsigned char Buffer1[30]={0};
unsigned char SETZERO=0;
//剩余电量查询指令
const uint8_t Current_Charge_Of_Battery[Length]= {
  0x4E,0x57,0x00,0x13,0x00,
  0x00,0x00,0x00,0x03,0x03,
  0x00,0x85,0x00,0x00,0x00,
  0x00,0x68,0x00,0x00,0x01,
  0xAB
};
//查询电流
const uint8_t Check_I[Length]= {
  0x4E,0x57,0x00,0x13,0x00,
  0x00,0x00,0x00,0x03,0x03,
  0x00,0x84,0x00,0x00,0x00,
  0x00,0x68,0x00,0x00,0x01,
  0xAA
};
int Pub_Battoty();
int Pub_I();

int main(int argc,char**argv)
{
  ros::init(argc,argv,"Battory");
  ros::NodeHandle nh;
  ros::Publisher pub= nh.advertise<msg::Battory>("Bot_Battory",8);
  ros::param::get("~Ports",Ports);
  try
  {
    m_serial.setPort(Ports);
    m_serial.setBaudrate(baudrate);
    serial::Timeout Wait_Time = serial::Timeout::simpleTimeout(1000);
    m_serial.setTimeout(Wait_Time);
    m_serial.open();
  }
  catch(serial::IOException &info)
  {
    ROS_ERROR_STREAM("Unable to open Ports :"<<Ports);
        return -1;
  }

  if (m_serial.isOpen())
  {
    ROS_INFO_STREAM("Serial Ports initialized:"<<Ports);
  }
  else
  {
      return -1;
  }
  while (ros::ok())
  {
    memset(Buffer1, SETZERO,30);//清除缓存
    usleep(10);
   m_Battory.Rest_Battory=Pub_Battoty();
    m_Battory.InOrOut=Pub_I();
   pub.publish(m_Battory);
    sleep(1);
  }
}
//------------------电流函数
//1-in 0-out
int Pub_I()
{
  memset(Buffer1, SETZERO,30);//清除缓存
  if(m_serial.available())
  {
    ROS_INFO("no data");
  }
  m_serial.write(Check_I,Length);
  usleep(10);
  m_serial.read(Buffer1,13);

  m_serial.flushInput();
  if(0x80==(0x80&Buffer1[12]))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
//-----------------电量函数
int Pub_Battoty()//返回电量
{
   memset(Buffer1, SETZERO,30);

  if(m_serial.available())
  {
    ROS_INFO("no data");
  }

  m_serial.write(Current_Charge_Of_Battery,Length);
  usleep(100);
  m_serial.read(Buffer1,13);
  m_serial.flushInput();
  int left = Buffer1[12];
  sleep(1);
  return left;
}


