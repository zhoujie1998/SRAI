/*
主要功能：记录多个点位信息，生成指定名称的yaml文件
方法：订阅/Joy，在回调函数中当指定按键被按下时，才将当时的/odom点位信息记录下来
编译注意：需要添加yaml-cpp库
target_link_libraries(make_MapYaml_node
  ${catkin_LIBRARIES}
  yaml-cpp
 )
作者 Peter 联系方式 1027120908@qq.com
 */
#include "ros/ros.h"
#include "yaml-cpp/yaml.h"
#include "fstream"
#include "sensor_msgs/Joy.h"
#include "nav_msgs/Odometry.h"
using namespace std;
int number=1;        //初始化 点位数量
string line_name;     //路线名
class make_mapyaml:public fstream     //继承fstream，见头文件
{
public:
  make_mapyaml();
  ~make_mapyaml();
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg);   //Joy回调函数
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);  //odom回调函数
protected:
  ros::NodeHandle nh;
  YAML::Node map;
  ros::Subscriber sub_joy;
  ros::Subscriber sub_odom;
  string positionX_,positionY_,positionZ_,orientationX_,orientationY_,orientationZ_,orientationW_;
  float positionx,positiony,positionz,orientationx,orientationy,orientationz,orientationw;
};
//构造函数
make_mapyaml::make_mapyaml()
{
  printf("请输入路线名字\n");           //键盘输入流赋值给 路线名line_name
  printf("line_name=");
  cin>>line_name;
  map["Target_Point_Number"]=number;   //将点位数量赋值给yaml参数  Target_Point_Number
  sub_odom = nh.subscribe("/odom",100,&make_mapyaml::odomCallback,this);   //订阅/odom里的点位信息
  sub_joy = nh.subscribe("/joy",100,&make_mapyaml::joyCallback,this);      //订阅/Joy，在回调函数中当指定按键被按下时，才将当时的点位信息记录下来
}
//析构函数
make_mapyaml:: ~make_mapyaml()
{
  map["Target_Point_Number"]=--number;   //更新 参数Target_Point_Number
  //待修改 路径可以读取当前cpp的当前路径
  ofstream fout("/home/srai/srai_ws/src/test_navigation/param/"+line_name+".yaml");   //设置点位文件位置及名称
  fout<<map<<endl;                  //写入map组的信息到yaml文件
  fout.close();                     //关闭文件
}

//odom回调函数
void make_mapyaml::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    positionx=odom_msg->pose.pose.position.x;
    positiony=odom_msg->pose.pose.position.y;
    positionz=odom_msg->pose.pose.position.z;
    orientationx=odom_msg->pose.pose.orientation.x;
    orientationy=odom_msg->pose.pose.orientation.y;
    orientationz=odom_msg->pose.pose.orientation.z;
    orientationw=odom_msg->pose.pose.orientation.w;
}
//Joy回调函数
void make_mapyaml::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
  if(joy_msg->buttons[7]==true)   //指定手柄哪一个键是收集确认键
  {
    printf("number=%d\n",number);
    //给不同的点位依次加上标号
    positionX_="positionX_"+std::to_string(number);
    positionY_="positionY_"+std::to_string(number);
    positionZ_="positionZ_"+std::to_string(number);
    orientationX_="orientationX_"+std::to_string(number);
    orientationY_="orientationY_"+std::to_string(number);
    orientationZ_="orientationZ_"+std::to_string(number);
    orientationW_="orientationW_"+std::to_string(number);
    //将具体的odom数据传给对应的参数
    map[line_name][positionX_]=positionx;
    map[line_name][positionY_]=positiony;
    map[line_name][positionZ_]=positionz;
    map[line_name][orientationX_]=orientationx;
    map[line_name][orientationY_]=orientationy;
    map[line_name][orientationZ_]=orientationz;
    map[line_name][orientationW_]=orientationw;
    number++;      //记录点位数量加1
  }
}
int main(int argc,char **argv)
{
//  ros::init(argc,argv,"make_yaml");
//  make_mapyaml oA;
//  ros::spin();
  return 0;
}
