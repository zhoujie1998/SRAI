/***********************
 * 该节点为轮毂电机驱动节点
 * 为保证机器人程序的各项进程出现异常后，该节点可以及时向轮毂电机发送停止命令。
 * 异常信号待完善....
************************/
#include "ros/ros.h"
#include "modbus/modbus-rtu.h"
#include "msg/two_motor_speed.h"
#include "std_msgs/Bool.h"
#include "signal.h"   //signal函数头文件
using namespace std;
modbus_t *ctx=NULL;

msg::two_motor_speed translate_ctx_data(msg::two_motor_speed msg)
{
 uint16_t ctxbuffer_l=0,ctxbuffer_r=0;
 modbus_read_registers(ctx,0x20AB,1,&ctxbuffer_l);
 usleep(10);
 modbus_read_registers(ctx,0x20AC,1,&ctxbuffer_r);

 //左轮正负数处理
 if (ctxbuffer_l>=0x8000)
 {
   msg.two_motor_speed[0]=(ctxbuffer_l-0x10000)/10;
 }
 else
 {
   msg.two_motor_speed[0]=ctxbuffer_l/10;
 }
 //右轮正负数处理
 if (ctxbuffer_r>=0x8000)
 {
   msg.two_motor_speed[1]=(ctxbuffer_r-0x10000)/10;
 }
 else
 {
   msg.two_motor_speed[1]=ctxbuffer_r/10;
 }
 return msg;
}

void twistCallback(const msg::two_motor_speed & mycmd)
{
 modbus_write_register(ctx,0x2088,mycmd.two_motor_speed[0]);//左轮
 modbus_write_register(ctx,0x2089,mycmd.two_motor_speed[1]);//右轮
}

//异常信号处理函数
void mySigIntHandler(int sig)
{
  switch (sig) {
      case SIGINT:
        ROS_INFO("process exit: SIGINT: value: %d\n",sig);   //用户按键请求，如按下Ctrl+C键
        break;
      case SIGFPE:
        ROS_ERROR("process exit: SIGFPE: value: %d\n",sig);  //程序异常中止，如调用abort函数
        break;
      default:
        //其他异常情况，暂不对底盘轮子速度处理，直接ros::shutdown()后退出
        ROS_INFO("process exit: SIGFPE: value: %d\n",sig);
        ros::shutdown();
        return;
   }
   printf("close ZLAC the serial\n");
   uint16_t data[2]={0x0000,0x0000};
   //轮子速度0同步下发，成功后延时20ms关闭modbus，否则延时20ms后继续
   while(modbus_write_registers(ctx,0x2088,2,data)==-1)
   {
     usleep(1000*20);
     continue;
   }
   usleep(1000*20);
   modbus_close(ctx);
   modbus_free(ctx);
   ros::shutdown();
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "two_ZLAC8015D_485");

  ros::NodeHandle nh;
  std::string Ports;
  ros::param::get("~Ports",Ports);
  ros::Publisher car_data_pub=nh.advertise<msg::two_motor_speed>("/motor/feedback_speed", 100);
  ros::Publisher stop_pub = nh.advertise<std_msgs::Bool>("/stop",100);
  ros::Subscriber car_control_sub=nh.subscribe("/motor/cmd_speed",100,twistCallback);

  //初始化RTU指针
  if(!(ctx=modbus_new_rtu(Ports.c_str(),115200,'N',8,1)))
    {
      printf("初始化modbus指针出错，端口号不匹配\n");
      modbus_free(ctx);
      ROS_FATAL("%s", modbus_strerror(errno));
      return 0;
    }
     //设置从机地址
    if(modbus_set_slave(ctx,1)==-1)
    {
       modbus_free(ctx);
       return 0;
    }
    //设置等待时间，超过时间没连接上则报错
    modbus_set_response_timeout(ctx,0,200000);//超时时间
    //建立主从机连接
    if(modbus_connect(ctx)==-1)
    {
       ROS_ERROR("can't connect /dev/ZLAC");
       modbus_free(ctx);
       return 0;
    }


    std_msgs::Bool stop;
    uint16_t ctxbuffer_state=0;
    msg::two_motor_speed msg;
    ros::Rate loop_rate(20);

   //异常信号处理函数
    signal(SIGINT, mySigIntHandler);
    signal(SIGABRT,mySigIntHandler);

     while (ros::ok())
     {
        //驱动器状态
        if(modbus_read_registers(ctx,0x20A2,1,&ctxbuffer_state)==1)
        {
          switch (ctxbuffer_state) {
          case 0x0000:
            cout<<"轮毂电机解轴"<<endl;
            break;
          case 0x8080:
            {
              cout<<"轮毂电机急停"<<endl;
              ROS_ERROR("The motor is really alarming!");
              stop.data=true;
            };break;
          case 0xc0c0:
            cout<<"轮毂电机报警"<<endl;
            ROS_ERROR("The motor is really alarming!");
            break;
          default:
            {
              //cout<<"轮毂电机锁轴"<<endl;
              car_data_pub.publish(translate_ctx_data(msg));
              stop.data=false;
              break;
            }
          }
          stop_pub.publish(stop);
        }
        else {
          ROS_ERROR(" can't connect /dev/ZLAC ");
        }
        loop_rate.sleep();
        ros::spinOnce();
      }
     modbus_close(ctx);
     modbus_free(ctx);
     return 0;
  }

