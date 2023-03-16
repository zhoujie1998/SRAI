#include "ros/ros.h"
#include"geometry_msgs/Twist.h"
#include"sensor_msgs/Joy.h"
ros::Publisher  pub;
ros::Subscriber sub;
geometry_msgs::Twist twist_msg;

double max_linear_x = 0.3;
int liear_x_key,angular_z_key,add_key,reduce_key;
void chatterCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
 {
  //当收到手柄加档按键信号且最大速度小于1.2m/s,最大线速度增加0.3m/s
  if(joy_msg->buttons[add_key]==true && max_linear_x<1.2)
  {
    max_linear_x=max_linear_x+0.3;
    ROS_INFO("max_linear.x=%.2f m/s",max_linear_x);
  }
  //当收到手柄减档按键信号且最大速度大于0.3m/s,最大线速度减小0.3m/s
  if(joy_msg->buttons[reduce_key]==true && max_linear_x>0.3)
  {
    max_linear_x=max_linear_x-0.3;
    ROS_INFO("max_linear.x=%.2f m/s",max_linear_x);
  }
  //最大线速度根据档位变化，最大角速度为固定值0.5m/s
  twist_msg.linear.x  = joy_msg->axes[liear_x_key]*max_linear_x;
  twist_msg.angular.z = joy_msg->axes[angular_z_key]*0.5;
}
int main(int argc,char **argv)
{
  ros::init(argc,argv,"sr_joy");
  ros::NodeHandle n;
  int frequency;
  std::string topic_cmd_vel;
  ros::param::get("~add_key",add_key);
  ros::param::get("~reduce_key",reduce_key);
  ros::param::get("~liear_x_key",liear_x_key);
  ros::param::get("~angular_z_key",angular_z_key);
  ros::param::get("~frequency",frequency);
  ros::param::get("~topic_cmd_vel",topic_cmd_vel);
  pub = n.advertise<geometry_msgs::Twist>(topic_cmd_vel,100);
  sub = n.subscribe("/joy",1000,chatterCallback);
  ros::Rate loop(frequency);
  bool stop=false;    //手柄数据非0标志位  false:数据为非0  true：数据为0
  while(ros::ok())
   {
    //手柄未拨动摇杆，joy_node节点持续发布数据为0的/joy话题;
    //增加该判断可以在导航move_base节点发布cmd_vel话题的同时，sr_joy_node节点不发布cmd_vel话题
    //当需要手柄控制底盘时，只要保证该frequency频率远大于move_base发布的cmd_vel即可
    if(stop==true && fabs(twist_msg.linear.x) == 0.0 && fabs(twist_msg.angular.z) == 0.0) {
      stop=false;
      loop.sleep();
      ros::spinOnce();
      continue;
    }
    if(stop==false && fabs(twist_msg.linear.x) == 0.0 && fabs(twist_msg.angular.z) == 0.0) stop=true;
    pub.publish(twist_msg);
    loop.sleep();
    ros::spinOnce();
   }
  return 0;
}
