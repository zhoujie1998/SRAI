/*"msg/sr_sensor
 舵机ros驱动
数据第一位  帧头  0X01
数据第二位  16进制数据位 高位
数据第三位  16进制数据位 低位
数据第四位  校验位  数据位相加
*/
#include "ros/ros.h"
#include "serial/serial.h"
#include "msg/sr_sensor.h"
#include "geometry_msgs/Twist.h"
#include "signal.h"
#include "math.h"
#define Base_Width 0.9  //轴距  单位米
#define HEADER 0xff     //数据头
#define ZERO 128        //轮子摆幅零度的舵机数据
#define l 130
using namespace std;
serial::Serial ser; //声明串口对象
//数据联合体
union Data
{
   uint8_t data[4];
   struct
   {
      uint8_t Header;
      uint8_t DataH;
      uint8_t DataL;
      uint8_t Check;
   }prot;
}Data;

static void Datatransmission(int16_t data)
{
  data=ZERO-data;
  printf("%d\n",data);
  if(data>254||data<0)    //对舵机接受的度数进行限制
  {
    if(data>254)
      data=254;
    if(data<0)
      data=0;
  }
  Data.prot.Header = HEADER;
  Data.prot.DataH = 0;
  Data.prot.DataL = data;
  Data.prot.Check = Data.prot.DataH+Data.prot.DataL;
  ser.write(Data.data,sizeof(Data.data));
}

void cmd_velCallback(const geometry_msgs::TwistConstPtr &twist_msgs)
{
  int16_t data;
  double R;

  R=twist_msgs->linear.x/twist_msgs->angular.z;
  if(isnormal(R)!=1||R==0) {
    data=0;
  }
  else {
    if(R<1.8&&R>0)
      R=1.8;
    if(R>-1.8&&R<0)
      R=-1.8;
    data=(180/M_PI)*atan(1/R)/0.21;
  }
  Datatransmission(data);
}
//******节点关闭函数*******//
void mySigIntHandler(int sig)
{
   cout<<"关闭节点!"<<endl;
   ros::shutdown();
}
//********************************************************************************************//
int main(int argc, char **argv)
{
  ros::init(argc,argv,"sr_steering_engine_node");
  signal(SIGINT, mySigIntHandler);

  ros::NodeHandle node;    //创建句柄，相当于一套工具，可以实例化 node，并且对 node 进行操作
  string Ports;
  node.param<string>("/sr_steering_engine_node/Ports",Ports,"/dev/ttyUSB0");
  //ros::Publisher pub = node.advertise<>("four_info",1);//创建 publisher 发布前轮转向角
  ros::Subscriber sub = node.subscribe("cmd_vel",1,cmd_velCallback);  //

  try
   {
        //设置串口属性，并打开串口
       ser.setPort(Ports);
       ser.setBaudrate(9600);
       serial::Timeout to = serial::Timeout::simpleTimeout(2000);
       ser.setTimeout(to);
       ser.open();
   }
   catch (serial::IOException& e)
   {
       cout<<"无法打开舵机串口"<<endl;
       return -1;
   }

   //检测串口是否已经打开，并给出提示信息
   if(ser.isOpen())
   {
       cout<<"舵机串口已初始化"<<endl;
   }
   else
   {
       return -1;
   }

   //开启20ms上传
  //Datatransmission(0);
  ros::spin();
  return 0;
}
