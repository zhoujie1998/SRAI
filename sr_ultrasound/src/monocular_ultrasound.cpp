
/*******************************************
********电应普 单目超声波 A05 一拖四************
********************************************/
#include"ros/ros.h"
#include"iostream"
#include"serial/serial.h"
#include"string"
#include"msg/ultrasound.h"

msg::ultrasound serial_redbuff(serial::Serial &myserial)
{
  uint8_t red_buff[10]={0};
  static msg::ultrasound mydata;
  size_t n=myserial.read(red_buff,sizeof (red_buff));
  uint8_t SUM=0;
  for (uint8_t i=0;i<sizeof (red_buff)-1;i++) {
    SUM=red_buff[i]+SUM;
  }
  if(n==sizeof (red_buff) && red_buff[0]==0xff && SUM==red_buff[sizeof (red_buff)-1])
  {
     for (uint i=0;i<4;i++) {
       mydata.ultrasound_data[i]=(red_buff[i*2+1]*256+red_buff[i*2+2]);
       //由于A05检测不到物体或者超量程上报数据为0,
       //当机器人前方空旷时这个数据0会干扰cost_map超声波层的障碍物数据
       if (mydata.ultrasound_data[i] == 0)
         mydata.ultrasound_data[i] = 4000;
     }
  }
  return mydata;
}

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"monocular_ultrasound_node");
  ros::NodeHandle np;
  ros::Publisher pub_ultrasound=np.advertise<msg::ultrasound>("/ultrasound_data",50);
  std::string Ports;
  ros::param::get("~Ports",Ports);
  serial::Serial ultrasound;
  ultrasound.setPort(Ports);
  ultrasound.setBaudrate(9600);
  serial::Timeout to=serial::Timeout::simpleTimeout(100);
  ultrasound.setTimeout(to);
  try {
    ultrasound.open();
  } catch (serial::IOException &e) {
    printf("打开失败");
    std::cout<<"---"<<Ports+" open erro"<<std::endl;
    return -1;
  }
  if(ultrasound.isOpen())
    std::cout<<Ports+" open OK"<<std::endl;
      //200ms循环一次
  ros::Rate red_sleep(5);
  msg::ultrasound myserial_data;
  while (ros::ok())
  {
    myserial_data=serial_redbuff(ultrasound);
    pub_ultrasound.publish(myserial_data);
    red_sleep.sleep();
  }
  return 0;
}
