/*
#int16[0] 前左轮转速
#int16[1] 前左轮转速
#int16[2] 中左轮转速
#int16[3] 中左轮转速
#int16[4] 后左轮转速
#int16[5] 后左轮转速

8030L的switch拨杆均在上，可自定义驱动器地址从4开始，本代码可通过i=?,和DriverEND来控制主机和哪些从机通信

为了实现modbus可以在主程序循环modbus_read_registers读取电机转速
 并且/cmd_vel发布节点死亡后，modbus中的下发速度可以立即停止
 实现方法：通过给/cmd_vel回调函数设置单独的回调函数队列，通过检查队列是否为空来判度/cmd_vel发布者是否死亡
*/
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/callback_queue_interface.h"
#include "ros/message.h"
#include "modbus/modbus-rtu.h"
#include "msg/six_motor_speed.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "signal.h"
//#include "boost/thread.hpp"
#define DriverEND 5   //循环控制至最后的驱动器地址
using namespace std;

class ModbusZL8030L
{
public:
  ModbusZL8030L();       //构造函数
  ~ModbusZL8030L();      //析构函数
  bool OpenSerial();     //打开串口函数
  void FeedBackSpeed();  //电机转速反馈函数
  void Controlset();     //设置电机控制模式函数
  void FaultCheck();     //故障检查函数
  void TwistCallback(const geometry_msgs::Twist::ConstPtr &msg);      //车辆速度控制/cmd_vel回调函数
  void EmergencyStopCallback(const std_msgs::Bool::ConstPtr &msg);    //车辆急停控制/emergency_stop回调函数
  public:
  ros::NodeHandle node;
  ros::NodeHandle nh;
  ros::CallbackQueue my_callback_queue;   //自定义消息队列
  modbus_t *ctx[11]={nullptr};
  ros::Publisher six_speed_pub,pub_motor_speed;
  ros::Subscriber sub_cmd_vel,sub_emergency_stop;
  msg::six_motor_speed SixSpeed;
  msg::six_motor_speed six_motor_speed;
  double Vr,Vl,vr,vl;
  double w_s,w_r;         //w_r 车轮半径
  bool is_connected;      //上位机与驱动器连接通讯标志位
  bool is_into_TwistCallback;
  string Ports;           //用于存放串口名称
};

//**********构造函数**********************************************************//
ModbusZL8030L::ModbusZL8030L()
{
  //将nh节点句柄的消息队列设为自定义消息队列my_queue
  nh.setCallbackQueue(&my_callback_queue);
  sub_cmd_vel=nh.subscribe("/cmd_vel",1,&ModbusZL8030L::TwistCallback,this);     //句柄为nh，单独回调函数队列

 // sub_cmd_vel=node.subscribe("/cmd_vel",1,&ModbusZL8030L::TwistCallback,this);  //句柄为node，通用回调函数队列
  pub_motor_speed=node.advertise<msg::six_motor_speed>("/motor/speed",1);
  sub_emergency_stop=node.subscribe("/emergency_stop",100,&ModbusZL8030L::EmergencyStopCallback,this);
  node.param<string>("/ZLAC8030L_485_modbus_node/Ports",Ports,"/dev/ttyUSB0");
  is_connected=false;
  is_into_TwistCallback=my_callback_queue.empty();
  if(OpenSerial()==true) {
    cout<<"轮毂电机驱动器，串口正确打开\n"<<endl;
    //Controlset();    //设置电机工作模式，默认为速度模式
    FeedBackSpeed();   //电机速度反馈函数
  }
  else {
    cout<<"轮毂电机驱动器，串口打开失败\n"<<endl;
    return;
  }
}

//**************打开串口函数******************************************************//
bool ModbusZL8030L::OpenSerial()
{
  cout<<"串口初始化及驱动器检查"<<endl;
  for (int i=4;i<=DriverEND;i++) {
    //初始化RTU指针
    ctx[i]=modbus_new_rtu(Ports.c_str(),115200,'N',8,1);
    if(ctx[i]==NULL) {
      printf("初始化modbus指针出错，端口号不匹配\n");
      modbus_free(ctx[i]);
      ROS_FATAL("%s", modbus_strerror(errno));
      return false;
    }

    //设置串口模式
    modbus_rtu_set_serial_mode(ctx[i],MODBUS_RTU_RS485);

    //设置从机地址4、5、6、7、8、9
    if(modbus_set_slave(ctx[i],i)==-1) {
       printf("设置从机地址，寄存器%d，失败\n",i);
       modbus_free(ctx[i]);
       ROS_FATAL("%s", modbus_strerror(errno));
       return false;
    }
    else {
      //printf("设置从机地址，寄存器%d，成功\n",i);
    }

    //设置等待时间，超过时间没连接上则报错
    if(modbus_set_response_timeout(ctx[i],0,200000)==-1) {
      printf("超过等待时间，寄存器%d\n",i);
      ROS_FATAL("%s", modbus_strerror(errno));
      modbus_free(ctx[i]);
      return false;
    }
    else {
      //printf("设置等待时间，寄存器%d，成功\n",i);
    }

    //建立主从机连接
    if(modbus_connect(ctx[i])==-1) {
      printf("建立从机连接，寄存器%d，失败\n",i);
      ROS_FATAL("%s", modbus_strerror(errno));
      modbus_free(ctx[i]);
      return false;
    }
    else {
      //printf("建立从机连接，寄存器%d，成功\n",i);
    }

    //读取电机转速
    uint16_t dest_model[DriverEND];
    if(modbus_read_registers(ctx[i],0x202C,1,&dest_model[i])==-1) {
      ROS_FATAL("%s", modbus_strerror(errno));
      modbus_free(ctx[i]);
      return false;
    }
    else{
      printf("读取电机转速，寄存器%d，成功\n",i);
    }
  }
  return true;
}

//*********电机控制模式设置**********************************************************//
void ModbusZL8030L::Controlset()
{
  int value=3;//1相对位置模式 2绝对位置摸索 3速度模式 4转矩模式
  for (int i=4;i<=DriverEND;++i) {
    modbus_write_register(ctx[i],0x2032,value);
  }
  printf("设置电机控制模式为速度模式\n");
}

//*********故障检查函数************//
void ModbusZL8030L::FaultCheck()
{
  cout<<"轮毂电机驱动器故障检查"<<endl;
  for (int i=4;i<=DriverEND;i++) {
    uint16_t *dest;           //用于存放驱动器故障码
    if(modbus_read_registers(ctx[i],0x202C,1,dest)==1) {
      switch (*dest) {
      case 0x0000:
        cout<<"驱动器"<<i<<"，无错误"<<endl;break;
      case 0x0001:
        cout<<"驱动器"<<i<<"，过压"<<endl;break;
      case 0x0002:
        cout<<"驱动器"<<i<<"，欠压"<<endl;break;
      case 0x0004:
        cout<<"驱动器"<<i<<"，过流"<<endl;break;
      case 0x0008:
        cout<<"驱动器"<<i<<"，过载"<<endl;break;
      case 0x0010:
        cout<<"驱动器"<<i<<"，电流超差"<<endl;break;
      case 0x0020:
        cout<<"驱动器"<<i<<"，编码器超差"<<endl;break;
      case 0x0040:
        cout<<"驱动器"<<i<<"，速度超差"<<endl;break;
      case 0x0080:
        cout<<"驱动器"<<i<<"，参考电压出错"<<endl;break;
      case 0x0100:
        cout<<"驱动器"<<i<<"，EEPROM读写错误"<<endl;break;
      case 0x0200:
        cout<<"驱动器"<<i<<"，霍尔出错"<<endl;break;
      case 0x0400:
        cout<<"驱动器"<<i<<"，电机温度过高"<<endl;break;
      }
    }
    else {
      cout<<"读取驱动器"<<i<<"故障码失败，主从机连接错误"<<endl;
      ROS_FATAL("%s", modbus_strerror(errno));
    }
  }
}

//*********反馈速度设置************************************************************************//
void ModbusZL8030L::FeedBackSpeed()
{
  ros::Rate loop_rate(50);
  while (ros::ok()) {
    //监听自定义回调函数消息队列
    if(is_into_TwistCallback==false&&my_callback_queue.empty()==true) {
      cout<<"发布速度队列为空"<<endl;
      usleep(1000*1000);
      is_into_TwistCallback=true;
      continue;
    }

    for (int i=4;i<=DriverEND;++i) {
      //SixSpeed.Speed[i]=0; //默认不置零
      uint16_t dest_model[1]={0};
      usleep(1000*20);
      if(modbus_read_registers(ctx[i],0x202C,1,dest_model)==-1) {
        cout<<"电机"<<i<<"速度反馈,错误"<<endl;
        ROS_ERROR("%s", modbus_strerror(errno));
      }
      else {
        SixSpeed.Speed[i-4]=dest_model[0];
      }
    }
    pub_motor_speed.publish(SixSpeed);
    //处理自定义回调函数队列中一个元素就返回
    my_callback_queue.callOne(ros::WallDuration());
//    处理完自定义回调函数队列中所有元素再返回
//    ros::AsyncSpinner spinner(0, &my_callback_queue);
//    spinner.start();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

//*********急停/emergency_stop话题回调函数**********************************************************//
void ModbusZL8030L::EmergencyStopCallback(const std_msgs::Bool::ConstPtr &msg)
{
  if(msg->data==true) {
    cout<<"启动急停"<<endl;
    for (int i=4;i<=DriverEND;++i) {
      modbus_write_register(ctx[i],0x2031,0x0005); //急停
    }
  }
  else {
    cout<<"解除急停"<<endl;
    for (int i=4;i<=DriverEND;++i) {
      modbus_write_register(ctx[i],0x2031,0x0008); //解除急停
    }
  }
}

//********运动学反解************************************************************************************//
void ModbusZL8030L::TwistCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  is_into_TwistCallback=my_callback_queue.empty();
  w_s=1;
  w_r=0.3;
  //Vr=msg->linear.x+(msg->angular.z*w_s/2.0);
  //Vl=msg->linear.x-(msg->angular.z*w_s/2.0);
  int R;
  R=msg->linear.x/msg->angular.z;
  six_motor_speed.Speed[0]=-60*msg->linear.x/(2.0*M_PI*w_r)*(R/sqrt((R-0.45)*(R-0.45)+1.0*1.0));
  six_motor_speed.Speed[1]=60*msg->linear.x/(2.0*M_PI*w_r)*(R/sqrt((R+0.45)*(R+0.45)+1.0*1.0));

  six_motor_speed.Speed[2]=-60*msg->linear.x/(2.0*M_PI*w_r)*(R/sqrt((R-0.45)*(R-0.45)+0.3*0.3));
  six_motor_speed.Speed[3]=60*msg->linear.x/(2.0*M_PI*w_r)*(R/sqrt((R+0.45)*(R+0.45)+0.3*0.3));
  
  six_motor_speed.Speed[4]=six_motor_speed.Speed[2];
  six_motor_speed.Speed[5]=six_motor_speed.Speed[3];

  for (int i=4;i<=DriverEND;i++) {
    //写单个数据到寄存器
    //modbus_write_register(ctx[i],0x203A,six_motor_speed.Speed[i]);
    //写多个数据到寄存器
    //uint16_t speed[2]={0X0010,0XFF10};
    //modbus_write_registers(ctx[i],0x203A,1,speed);

    if(modbus_write_register(ctx[i],0x203A,six_motor_speed.Speed[i])==-1) {
      ROS_ERROR("%s", modbus_strerror(errno));
      printf("写入数据到寄存器%d，出错\n",i);
      return;
    }
    else {
     // printf("写入数据到寄存器%d，成功\n",i);
    }
  }
  cout<<"退出回调函数"<<endl;
}

//****************析构函数*************************************************************************//
ModbusZL8030L::~ModbusZL8030L()
{
  FaultCheck();    //故障检查

  for (int i=4;i<6;i++) {
    modbus_close(ctx[i]);  //关闭主从连接
    modbus_free(ctx[i]);   //释放一个已分配的modbus_t结构体。
  }
}

void FeedBackSpeed()
{

}

//**********节点关闭函数****************************************************************************//
void mySigIntHandler(int sig)
{
   printf("关闭节点!\n");
   ros::shutdown();
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"ZLAC8030L_485_modbus");
  signal(SIGINT, mySigIntHandler);//把连接到mySigIntHandler保证关闭节点时能够关闭20ms数据上传
  ModbusZL8030L modbus;
  //多线程
  //boost::thread server(boost::bind(&ModbusZL8030L::FeedBackSpeed,&modbus));
  //server.join();
  ros::spin();
  return 0;
}
