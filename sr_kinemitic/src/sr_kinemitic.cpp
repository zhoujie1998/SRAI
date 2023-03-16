#include "ros/ros.h"
#include "msg/two_motor_speed.h"
#include "geometry_msgs/Twist.h"
class kinematics
{
public:
  kinematics()
  {
    ros::NodeHandle nh;
    ros::param::get("~wheel_spacing",wheel_S);
    ros::param::get("~wheel_radius",wheel_R);
    pub_motorcmdspeed=nh.advertise<msg::two_motor_speed>("/motor/cmd_speed",100);
    sub_motorfeedbackspeed=nh.subscribe("/motor/feedback_speed",100,&kinematics::Callback,this);
    pub_feedbackcmdvel=nh.advertise<geometry_msgs::Twist>("/feedback/cmd_vel",100);
    sub_cmdvel=nh.subscribe("/cmd_vel",100,&kinematics::twistCallback,this);
  }
  void Callback(const msg::two_motor_speed::ConstPtr &msg);
  void twistCallback(const geometry_msgs::Twist::ConstPtr &mssg);
public:
  ros::Subscriber sub_motorfeedbackspeed;
  ros::Subscriber sub_cmdvel;
  ros::Publisher pub_motorcmdspeed;
  ros::Publisher pub_feedbackcmdvel;
  double RSpeed,LSpeed,RTwist,LTwist;
  double wheel_S,wheel_R;
};
void kinematics::twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  msg::two_motor_speed speed;
  LSpeed=msg->linear.x-(msg->angular.z*wheel_S/2.0);
  RSpeed=msg->linear.x+(msg->angular.z*wheel_S/2.0);
  //当电机逆时针旋转为正，车辆向前，右侧轮顺时针旋转，故加负号
  speed.two_motor_speed[0]=(60*LSpeed/(2.0*M_PI*wheel_R));
  speed.two_motor_speed[1]=-(60*RSpeed/(2.0*M_PI*wheel_R));
  pub_motorcmdspeed.publish(speed);
}
void kinematics::Callback(const msg::two_motor_speed::ConstPtr &msg)
{
  geometry_msgs::Twist V_Twist;
  //当电机逆时针旋转为正，车辆向前，右侧轮顺时针旋转，故加负号
  LTwist=(msg->two_motor_speed[0]/60.0*(2*M_PI*wheel_R));
  RTwist=-(msg->two_motor_speed[1]/60.0*(2*M_PI*wheel_R));
  V_Twist.linear.x=(LTwist+RTwist)/2;
  V_Twist.angular.z=(RTwist-LTwist)/wheel_S;
  pub_feedbackcmdvel.publish(V_Twist);
}
int main(int argc,char** argv)
{
  ros::init(argc,argv,"sr_kinematic");
  kinematics oA;
  ros::spin();
  return 0;
}
