/*
时代方舟DM055B驱动程序
*/
#include "ros/ros.h"
#include "modbus/modbus-rtu.h"
#include "msg/motor_speed.h"
#include "signal.h"
#define RDC 28//减速比

uint16_t bufferL[10];
uint16_t bufferR[10];
double speed[2];
msg::motor_speed my_data;
modbus_t *modbusL = NULL;
modbus_t *modbusR = NULL;

ros::Publisher car_data_pub;

void TranslateModbusData(void)
{
    double spL,spR;
    spL = (short)bufferL[0];
    spR = (short)bufferR[0];
   my_data.motor_speed[0] = spL/(10*RDC);//左轮

   my_data.motor_speed[1] = spR/(10*RDC);//右轮
   car_data_pub.publish(my_data);
}

void TwistCallback(const msg::motor_speed & mycmd)
{
  //当下发电机转速和上一次不变时，不下发
    if(speed[0] == mycmd.motor_speed[0] && speed[1] == mycmd.motor_speed[1]) return;

    speed[0] = mycmd.motor_speed[0];
    speed[1] = mycmd.motor_speed[1];
    int speed_l,speed_r;
    speed_l = speed[0]*RDC;
    speed_r = speed[1]*RDC;
    usleep(1000*10);
    int res = modbus_write_register(modbusL,0x02,speed_l);//左轮
    usleep(1000*10);
    int res1 = modbus_write_register(modbusR,0x02,speed_r);//右轮
}

//当关闭包时调用，当节点消亡之前给轮子下发速度0
void mySigIntHandler(int sig)
{
  ROS_INFO("close the modbus!\n");
  int data = 0;
  while(modbus_write_register(modbusL,0x02,data) == -1  ) {
    usleep(1000*10);
    continue;
  }
  usleep(1000*10);
  while(modbus_write_register(modbusR,0x02,data) == -1) {
  usleep(1000*10);
  continue;
  }
  modbus_close(modbusL);
  modbus_free(modbusL);
  modbus_close(modbusR);
  modbus_free(modbusR);
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fire_control_serial");
  ros::NodeHandle nh;
  std::string Ports;
  ros::param::get("~Ports",Ports);
  car_data_pub = nh.advertise<msg::motor_speed>("/motor/feedback_speed", 1000);
  ros::Subscriber car_control_sub=nh.subscribe("/motor/cmd_speed",1000,TwistCallback);

  //设置初始化RTU指针
  if(!(modbusL = modbus_new_rtu(Ports.c_str(),19200,'N',8,1)) || !(modbusR = modbus_new_rtu(Ports.c_str(),19200,'N',8,1))) {
    ROS_ERROR("can't init new rtu , please check the port:  %s.",Ports.c_str());
    modbus_free(modbusL);
    modbus_free(modbusR);
    ROS_FATAL("%s", modbus_strerror(errno));
    return 0;
  }

 //设置从机地址
  if((modbus_set_slave(modbusL,1) == -1) || (modbus_set_slave(modbusR,2) == -1))  {
    ROS_ERROR("can't set slave,please check  slaves.");
    modbus_free(modbusL);
    modbus_free(modbusR);
    ROS_FATAL("%s", modbus_strerror(errno));
    return 0;
  }

  //设置等待时间，超过时间没连接上则报错
  modbus_set_response_timeout(modbusL,0,200000);//超时时间
  modbus_set_response_timeout(modbusR,0,200000);//超时时间

  //建立主从机连接
  if((modbus_connect(modbusL) == -1) || (modbus_connect(modbusR) == -1)) {
    ROS_ERROR("can't connect %s",Ports.c_str());
    modbus_free(modbusL);
    modbus_free(modbusR);
    return 0;
  }
  ROS_INFO("Open  the port: %s ,Success.",Ports.c_str());
  ros::Rate loop_rate(20);
  signal(SIGINT, mySigIntHandler);  
  //查询电机转速 20HZ
  while (ros::ok()) {
  memset(bufferL,0,10*2);
  memset(bufferR,0,10*2);
  usleep(1000*10);
  if(modbus_read_registers(modbusL,0x10,1,bufferL) == 1) {
    usleep(1000*10);
    if(modbus_read_registers(modbusR,0x10,1,bufferR) == 1)
      TranslateModbusData();
  }
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
