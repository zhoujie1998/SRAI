#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
float x;
float y;
float z;

void chatterCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
  x=msg->axes[3];
  y=msg->axes[2];
  z=msg->axes[0];
}

int main(int argc,char **argv)
{
ros::init(argc,argv,"third_pkg");
ros::NodeHandle n;// 更新话题的消息格式为自定义的消息格式
ros::Publisher chatter_pub =n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);//发布话题
ros::Subscriber sub = n.subscribe("joy",1000,chatterCallback);//订阅 
ros::Rate loop_rate(10);
geometry_msgs::Twist msgg;


while(1)
{
msgg.linear.x=x;
msgg.angular.z=y;

chatter_pub.publish(msgg);
ros::spinOnce();
loop_rate.sleep();
}
//ros::spin();//阻塞回调
return 0;
}
