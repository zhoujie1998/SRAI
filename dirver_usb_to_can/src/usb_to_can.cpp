/*********************************
   can协议     控制电机
   电机速度反馈及报警信息设置为PDO自主上报，需要提前映射
   自主上报的开启和关闭，由NMT报文控制
   六轮车有多个从机，为了避免同一CAN通道数据传输数据过多，将前两轮置于通道1，后四轮置于通道2
*********************************/

#include "ros/ros.h"
#include "dirver_usb_to_can/controlcan.h"
#include "dirver_usb_to_can/speed.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/UInt16.h>
#define DriverEND  9        //循环控制至最后的驱动器地址4~9,驱动器地址拨杆均均为OFF
#define wheelR 0.23
using namespace std;
//union SixSpeed
//{
//  int32_t speed[6];
//  struct
//  {
//    int8_t FLSpeed;
//    int8_t FRSpeed;
//    int8_t WRSpeed;
//    int8_t WLSpeed;
//    int8_t BLSpeed;
//    int8_t BRSpeed;
//  }Speed;
//}SixSpeed;
class usb_to_can
{
public:
  usb_to_can();
  ~usb_to_can();   //在析构函数里有   关闭设备函数
  void TwistCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void CurrentCallback(const dirver_usb_to_can::speed::ConstPtr &current_msg);
  void FaultCheck();     //故障检查函数
  void Feedback();
  void listen_close();
  void listen_open();
  bool _VCI_OpenDevice();
  bool _VCI_InitCAN();
  bool _VCI_StartCAN();
  bool _VCI_CloseDevice();
protected:
  ros::NodeHandle nh;
  ros::Publisher pub,pub_current;
  ros::Subscriber sub_cmd_vel;
  dirver_usb_to_can::speed six_motor_speed;
  uint nDevType;
  uint nDevIndex;
  uint nCAN1Index,nCAN2Index;
  uint8_t Data[4];
  VCI_CAN_OBJ vco_Transmit[6];    //发送帧 电机转速
  VCI_CAN_OBJ vco_Receive_CAN1[1000],vco_Receive_CAN2[1000];   //接收帧 电机实际转速
  DWORD dwRel;
  VCI_INIT_CONFIG vic;
  double motor_speed[6];
};

usb_to_can::usb_to_can()
{
  nDevType= 4;
  nDevIndex= 0;
  nCAN1Index= 0;
  nCAN2Index= 1;

  vic.AccCode=0x80000008;
  vic.AccMask=0xFFFFFFFF;
  vic.Filter=1;       //滤波器同时对标准帧与扩展帧过滤!
  vic.Timing0=0x00;
  vic.Timing1=0x1C;   //波特率500KHz
  vic.Mode=0;         //正常工作模式

  if(_VCI_OpenDevice()!=true) {
    return;    //打开设备
  }

  if(_VCI_InitCAN()!=true) {
    return;    //初始化设备
  }

  if( _VCI_StartCAN()!=true) {
    return;    //启动设备
  }

  //发送帧数据  预设
  for (uint i=0;i<DriverEND-3;i++) {
    vco_Transmit[i].ID =0x604+i;   //节点地址  驱动器地址可由软件自定义 4～9
    vco_Transmit[i].RemoteFlag = 0;
    vco_Transmit[i].ExternFlag = 0;
    vco_Transmit[i].SendType= 1;
    vco_Transmit[i].DataLen = 8;    //数据长度
    vco_Transmit[i].Data[0]=0x23;   //命令字
    vco_Transmit[i].Data[1]=0xFF;   //索引低位
    vco_Transmit[i].Data[2]=0x60;   //索引高位
    vco_Transmit[i].Data[3]=0x00;   //子索引
    vco_Transmit[i].Data[4]=0x00;   //***低位***数据段
    vco_Transmit[i].Data[5]=0x00;   //****|****
    vco_Transmit[i].Data[6]=0x00;   //****V****
    vco_Transmit[i].Data[7]=0x00;   //***高位***数据段
  }
  pub_current=nh.advertise<dirver_usb_to_can::speed>("current",100);   //注册发布者，电机实时速度
  pub=nh.advertise<dirver_usb_to_can::speed>("motor_feedbackSpeed",100);   //注册发布者，电机实时速度
  sub_cmd_vel=nh.subscribe<geometry_msgs::Twist>("cmd_vel",100,&usb_to_can::TwistCallback,this); //注册订阅者，控制电机速度
  Feedback();          //反馈速度
}

//发布电机当前速度
void usb_to_can::Feedback()
{
  listen_open();
  ros::Rate loop_rate(5);
  dirver_usb_to_can::speed pub_six_speed;
  dirver_usb_to_can::speed pub_six_current;  //电机电流
  while(ros::ok())
  {
    //CAN通道1
    dwRel=VCI_Receive(nDevType, nDevIndex, nCAN1Index, vco_Receive_CAN1,1000,0);
    for (uint j=0;j<dwRel;j++) {
      //实际速度寄存器反馈地址 TDO映射至 184～185
      switch (vco_Receive_CAN1[j].ID) {
        case 0x184: {
          pub_six_speed.Speed[0]=(vco_Receive_CAN1[j].Data[0]+((vco_Receive_CAN1[j].Data[1]&0xffff)<<8)+((vco_Receive_CAN1[j].Data[2]&0xffffff)<<16)+((vco_Receive_CAN1[j].Data[3]&0xfffffffff)<<24));
          if((pub_six_speed.Speed[0])>=0x80000000)
            pub_six_speed.Speed[0]=-(((pub_six_speed.Speed[0]^0xffffffff)+0x00000001)/10.0);
          else
            pub_six_speed.Speed[0]=pub_six_speed.Speed[0]/10.0;
        }
          break;
        case 0x185: {
          pub_six_speed.Speed[1]=(vco_Receive_CAN1[j].Data[0]+((vco_Receive_CAN1[j].Data[1]&0xffff)<<8)+((vco_Receive_CAN1[j].Data[2]&0xffffff)<<16)+((vco_Receive_CAN1[j].Data[3]&0xfffffffff)<<24));
          if((pub_six_speed.Speed[1])>=0x80000000)
            pub_six_speed.Speed[1]=-(((pub_six_speed.Speed[1]^0xffffffff)+0x00000001)/10.0);
          else
            pub_six_speed.Speed[1]=pub_six_speed.Speed[1]/10.0;
        }
          break;
        case 0x284: {
          pub_six_current.Speed[0]=vco_Receive_CAN1[j].Data[0]+((vco_Receive_CAN1[j].Data[1]&0xffff)<<8);
          if((pub_six_current.Speed[0])>=0x8000)
            pub_six_current.Speed[0]=-((pub_six_current.Speed[0]^0xffff)+0x0001);
        }
          break;
        case 0x285: {
          pub_six_current.Speed[1]=vco_Receive_CAN1[j].Data[0]+((vco_Receive_CAN1[j].Data[1]&0xffff)<<8);
          if((pub_six_current.Speed[1])>=0x8000)
            pub_six_current.Speed[1]=-((pub_six_current.Speed[1]^0xffff)+0x0001);
        }
          break;
      default:
        break;
      }
    }

      //CAN通道2
      dwRel=VCI_Receive(nDevType, nDevIndex, nCAN2Index, vco_Receive_CAN2,1000,0);
      for (uint j=0;j<dwRel;j++) {
        printf("ID=%x\n",vco_Receive_CAN2[j].ID);
        //实际速度寄存器反馈地址 TDO映射至 186～189
        switch (vco_Receive_CAN2[j].ID) {
          case 0x186: {
            pub_six_speed.Speed[2]=(vco_Receive_CAN2[j].Data[0]+((vco_Receive_CAN2[j].Data[1]&0xffff)<<8)+((vco_Receive_CAN2[j].Data[2]&0xffffff)<<16)+((vco_Receive_CAN2[j].Data[3]&0xfffffffff)<<24));
            if((pub_six_speed.Speed[2])>=0x80000000)
              pub_six_speed.Speed[2]=-(((pub_six_speed.Speed[2]^0xffffffff)+0x00000001)/10.0);
            else
              pub_six_speed.Speed[2]=pub_six_speed.Speed[2]/10.0;
          }
            break;
          case 0x187: {
            pub_six_speed.Speed[3]=(vco_Receive_CAN2[j].Data[0]+((vco_Receive_CAN2[j].Data[1]&0xffff)<<8)+((vco_Receive_CAN2[j].Data[2]&0xffffff)<<16)+((vco_Receive_CAN2[j].Data[3]&0xfffffffff)<<24));
            if((pub_six_speed.Speed[3])>=0x80000000)
              pub_six_speed.Speed[3]=-(((pub_six_speed.Speed[3]^0xffffffff)+0x00000001)/10.0);
            else
              pub_six_speed.Speed[3]=pub_six_speed.Speed[3]/10.0;
          }
            break;
          case 0x188: {
            pub_six_speed.Speed[4]=(vco_Receive_CAN2[j].Data[0]+((vco_Receive_CAN2[j].Data[1]&0xffff)<<8)+((vco_Receive_CAN2[j].Data[2]&0xffffff)<<16)+((vco_Receive_CAN2[j].Data[3]&0xfffffffff)<<24));
            if((pub_six_speed.Speed[4])>=0x80000000)
              pub_six_speed.Speed[4]=-(((pub_six_speed.Speed[4]^0xffffffff)+0x00000001)/10.0);
            else
              pub_six_speed.Speed[4]=pub_six_speed.Speed[4]/10.0;
          }
            break;
          case 0x189: {
            pub_six_speed.Speed[5]=(vco_Receive_CAN2[j].Data[0]+((vco_Receive_CAN2[j].Data[1]&0xffff)<<8)+((vco_Receive_CAN2[j].Data[2]&0xffffff)<<16)+((vco_Receive_CAN2[j].Data[3]&0xfffffffff)<<24));
            if((pub_six_speed.Speed[5])>=0x80000000)
              pub_six_speed.Speed[5]=-(((pub_six_speed.Speed[5]^0xffffffff)+0x00000001)/10.0);
            else
              pub_six_speed.Speed[5]=pub_six_speed.Speed[5]/10.0;
          }
            break;
          case 0x286: {
            pub_six_current.Speed[2]=vco_Receive_CAN2[j].Data[0]+((vco_Receive_CAN2[j].Data[1]&0xffff)<<8);
            if((pub_six_current.Speed[2])>=0x8000)
              pub_six_current.Speed[2]=-((pub_six_current.Speed[2]^0xffff)+0x0001);
          }
            break;
          case 0x287: {
            pub_six_current.Speed[3]=vco_Receive_CAN2[j].Data[0]+((vco_Receive_CAN2[j].Data[1]&0xffff)<<8);
            if((pub_six_current.Speed[3])>=0x8000)
              pub_six_current.Speed[3]=-((pub_six_current.Speed[3]^0xffff)+0x0001);
          }
            break;
          case 0x288: {
            pub_six_current.Speed[4]=vco_Receive_CAN2[j].Data[0]+((vco_Receive_CAN2[j].Data[1]&0xffff)<<8);
            if((pub_six_current.Speed[4])>=0x8000)
              pub_six_current.Speed[4]=-((pub_six_current.Speed[4]^0xffff)+0x0001);
          }
            break;
          case 0x289: {
            pub_six_current.Speed[5]=vco_Receive_CAN2[j].Data[0]+((vco_Receive_CAN2[j].Data[1]&0xffff)<<8);
            if((pub_six_current.Speed[5])>=0x8000)
              pub_six_current.Speed[5]=-((pub_six_current.Speed[5]^0xffff)+0x0001);
          }
            break;
        default:
          break;
        }
    }
    pub.publish(pub_six_speed);
    pub_current.publish(pub_six_current);
    loop_rate.sleep();
    ros::spinOnce();
  }
}

//*************开启NMT报文*****************//
//若NMT报文开启失败，自主上报的PDO报文就不能自主上报
void usb_to_can::listen_open()
{
  //不同CAN通道的NMT报文分开
  VCI_CAN_OBJ listen_open_CAN1[2];
  VCI_CAN_OBJ listen_open_CAN2[4];
  for (uint8_t i=0;i<2;i++) {
    listen_open_CAN1[i].ID=0x000;
    listen_open_CAN1[i].TimeStamp=0;
    listen_open_CAN1[i].TimeFlag=0;
    listen_open_CAN1[i].SendType=1;
    listen_open_CAN1[i].RemoteFlag=0;
    listen_open_CAN1[i].ExternFlag=0;
    listen_open_CAN1[i].DataLen=2;
    listen_open_CAN1[i].Data[0]=0x01;
    listen_open_CAN1[i].Data[1]=0x04+i;
  }

  for (uint8_t i=0;i<4;i++) {
    listen_open_CAN2[i].ID=0x000;
    listen_open_CAN2[i].TimeStamp=0;
    listen_open_CAN2[i].TimeFlag=0;
    listen_open_CAN2[i].SendType=1;
    listen_open_CAN2[i].RemoteFlag=0;
    listen_open_CAN2[i].ExternFlag=0;
    listen_open_CAN2[i].DataLen=2;
    listen_open_CAN2[i].Data[0]=0x01;
    listen_open_CAN2[i].Data[1]=0x06+i;
  }
  VCI_Transmit(nDevType, nDevIndex, nCAN1Index,listen_open_CAN1,2);
  usleep(1000*10);
  VCI_Transmit(nDevType, nDevIndex, nCAN2Index,listen_open_CAN2,4);
}

//*************关闭NMT报文*****************//
void usb_to_can::listen_close()
{
  VCI_CAN_OBJ listen_close_CAN1[2];
  VCI_CAN_OBJ listen_close_CAN2[4];
  for (uint8_t i=0;i<2;i++) {
    listen_close_CAN1[i].ID=0x000;
    listen_close_CAN1[i].TimeStamp=0;
    listen_close_CAN1[i].TimeFlag=0;
    listen_close_CAN1[i].SendType=1;
    listen_close_CAN1[i].RemoteFlag=0;
    listen_close_CAN1[i].ExternFlag=0;
    listen_close_CAN1[i].DataLen=2;
    listen_close_CAN1[i].Data[0]=0x02;
    listen_close_CAN1[i].Data[1]=0x04+i;
  }

  for (uint8_t i=0;i<4;i++) {
    listen_close_CAN1[i].ID=0x000;
    listen_close_CAN1[i].TimeStamp=0;
    listen_close_CAN1[i].TimeFlag=0;
    listen_close_CAN1[i].SendType=1;
    listen_close_CAN1[i].RemoteFlag=0;
    listen_close_CAN1[i].ExternFlag=0;
    listen_close_CAN1[i].DataLen=2;
    listen_close_CAN1[i].Data[0]=0x02;
    listen_close_CAN1[i].Data[1]=0x06+i;
  }
  VCI_Transmit(nDevType, nDevIndex, nCAN1Index,listen_close_CAN1,2);
  usleep(1000*10);
  VCI_Transmit(nDevType, nDevIndex, nCAN2Index,listen_close_CAN2,4);
}

//控制电机转速，回调函数
void usb_to_can::TwistCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  double R;
  R=msg->linear.x/msg->angular.z; //转弯半径

  //运动学模型反解

  //判断R是否为nan或者inf
  if(isnormal(R)!=1) {
    motor_speed[0] = msg->linear.x;
    motor_speed[1] = msg->linear.x;
    motor_speed[2] = msg->linear.x;
    motor_speed[3] = msg->linear.x;
    motor_speed[4] = motor_speed[2];
    motor_speed[5] = motor_speed[3];
  }
  else {
    //限制转弯半径
    if(R<1.8&&R>0)
      R=1.8;
    if(R>-1.8&&R<0)
      R=-1.8;
    motor_speed[0]=msg->linear.x*fabs(sqrt((R-0.42)*(R-0.42)+1.03*1.03)/R);
    motor_speed[1]=msg->linear.x*fabs(sqrt((R+0.42)*(R+0.42)+1.03*1.03)/R);
    motor_speed[2]=msg->linear.x*fabs(sqrt((R-0.43)*(R-0.43)+0.27*0.27)/R);
    motor_speed[3]=msg->linear.x*fabs(sqrt((R+0.41)*(R+0.41)+0.27*0.27)/R);
    motor_speed[4]=msg->linear.x*fabs(sqrt((R-0.41)*(R-0.41)+0.27*0.27)/R);
    motor_speed[5]=msg->linear.x*fabs(sqrt((R+0.43)*(R+0.43)+0.27*0.27)/R);
  }
  //单位转换  m/s -> r/min
  for (int i=0;i<DriverEND-3;i++) {
    if(i%2==0) {     //左侧车轮
      six_motor_speed.Speed[i]=(int32_t)(-60*motor_speed[i]/(2.0*M_PI*wheelR));
    }
    else {           //右侧车轮
      six_motor_speed.Speed[i]=(int32_t)(60*motor_speed[i]/(2.0*M_PI*wheelR));
    }
  }

  //数据位分配
  for (uint i=0;i<DriverEND-3;i++) {
    vco_Transmit[i].Data[4]=(six_motor_speed.Speed[i]&0x000000ff);
    vco_Transmit[i].Data[5]=(six_motor_speed.Speed[i]&0x0000ff00)>>8;
    vco_Transmit[i].Data[6]=(six_motor_speed.Speed[i]&0x00ff0000)>>16;
    vco_Transmit[i].Data[7]=(six_motor_speed.Speed[i]&0xff000000)>>24;

    VCI_Transmit(nDevType, nDevIndex, nCAN1Index, vco_Transmit,2);
    VCI_Transmit(nDevType, nDevIndex, nCAN2Index, vco_Transmit,6);
    //printf("vco_Transmit[%d].ID=%x，%02x ,%02x ,%02x ,%02x ,%02x ,%02x, %02x, %02x\n",i,vco_Transmit[i].ID,vco_Transmit[i].Data[0],vco_Transmit[i].Data[1],vco_Transmit[i].Data[2],vco_Transmit[i].Data[3],vco_Transmit[i].Data[4],vco_Transmit[i].Data[5],vco_Transmit[i].Data[6],vco_Transmit[i].Data[7]);
  }
  //printf("vco_Transmit[0].ID=%x，%02x ,%02x ,%02x ,%02x \n",vco_Transmit[0].ID,vco_Transmit[0].Data[4],vco_Transmit[0].Data[5],vco_Transmit[0].Data[6],vco_Transmit[0].Data[7]);
  //printf("***************************************************\n");
}

//*********故障检查函数************//
void usb_to_can::FaultCheck()
{
  cout<<"轮毂电机驱动器故障检查"<<endl;
}

usb_to_can::~usb_to_can()
{
  listen_close();   //关闭NMT报文
  VCI_CloseDevice(nDevType, nDevIndex);      //关闭设备
}

bool usb_to_can::_VCI_OpenDevice()
{
  dwRel=VCI_OpenDevice(nDevType, nDevIndex, 0);
  if(dwRel != 1)
  {
    VCI_CloseDevice(nDevType, nDevIndex);
    cout<<"打开设备失败!"<<endl;
    return FALSE;
  }
}

bool usb_to_can::_VCI_InitCAN()
{
  dwRel=VCI_InitCAN(nDevType, nDevIndex, nCAN1Index, &vic);
  if(dwRel !=1)
  {
  VCI_CloseDevice(nDevType, nDevIndex);
  cout<<"初始化通道CAN1失败!"<<endl;
  return FALSE;
  }
  dwRel=VCI_InitCAN(nDevType, nDevIndex, nCAN2Index, &vic);
  if(dwRel !=1)
  {
  VCI_CloseDevice(nDevType, nDevIndex);
  cout<<"初始化通道CAN2失败!"<<endl;
  return FALSE;
  }
  //清空指定can通道缓存区
  VCI_ClearBuffer(nDevType,nDevIndex,nCAN1Index);
  VCI_ClearBuffer(nDevType,nDevIndex,nCAN2Index);
}


bool usb_to_can::_VCI_StartCAN()
{
  dwRel =VCI_StartCAN(nDevType, nDevIndex, nCAN1Index);
  if(dwRel !=1)
  {
  cout<<"启动通道CAN1失败,重试中"<<endl;
  dwRel=VCI_ResetCAN(nDevType, nDevIndex, nCAN1Index);
  if(dwRel !=1) {
     cout<<"重启动通道CAN1失败"<<endl;
    return FALSE;
  }
  }
  dwRel =VCI_StartCAN(nDevType, nDevIndex, nCAN2Index);
  if(dwRel !=1)
  {
  VCI_ResetCAN(nDevType, nDevIndex, nCAN2Index);
  cout<<"启动通道CAN2失败!,重试中"<<endl;
  dwRel=VCI_ResetCAN(nDevType, nDevIndex, nCAN2Index);
  if(dwRel !=1) {
     cout<<"重启动通道CAN2失败"<<endl;
    return FALSE;
  }
  }
}


bool usb_to_can::_VCI_CloseDevice()
{
  dwRel = VCI_CloseDevice(nDevType, nDevIndex);
  if(dwRel != 1) 
  {
  cout<<"关闭设备失败!"<<endl;
  return FALSE;
  }
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"usb_to_can");    //初始化ROS
  usb_to_can oA;                        //定义一个类的对象
  ros::spin();
  return 0;
}
