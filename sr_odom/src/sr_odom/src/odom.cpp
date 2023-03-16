/********************
 * 本节点根据轮子反馈速度话题时间戳的数据，发布TF树odom->base_footprint
 * 当轮子不再反馈速度时，认为TF树odom->base_footprint未发生变化
 *
 * 其他思路：以轮子最后一次发布的速度的时间戳的数据，发布TF树odom->base_footprint
 *
 */
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"

// 姿势协方差
boost::array<double, 36> ODOM_POSE_COVARIANCE = {
  {1e-3, 0, 0, 0, 0, 0,
   0, 1e-3, 0, 0, 0, 0,
   0, 0, 1e6, 0, 0, 0,
   0, 0, 0, 1e6, 0, 0,
   0, 0, 0, 0, 1e6, 0,
   0, 0, 0, 0, 0, 1e-3}};

// 转动协方差
boost::array<double, 36> ODOM_TWIST_COVARIANCE = {
  {1e-3, 0, 0, 0, 0, 0,
   0, 1e-3, 0, 0, 0, 0,
   0, 0, 1e6, 0, 0, 0,
   0, 0, 0, 1e6, 0, 0,
   0, 0, 0, 0, 1e6, 0,
   0, 0, 0, 0, 0, 1e-3}};

// 姿势协方差2
boost::array<double, 36> ODOM_POSE_COVARIANCE2 = {
  {1e-9, 0, 0, 0, 0, 0,
   0, 1e-3, 1e-9, 0, 0, 0,
   0, 0, 1e6, 0, 0, 0,
   0, 0, 0, 1e6, 0, 0,
   0, 0, 0, 0, 1e6, 0,
   0, 0, 0, 0, 0, 1e-9}};

// 转动协方差2
boost::array<double, 36> ODOM_TWIST_COVARIANCE2 = {
  {1e-9, 0, 0, 0, 0, 0,
   0, 1e-3, 1e-9, 0, 0, 0,
   0, 0, 1e6, 0, 0, 0,
   0, 0, 0, 1e6, 0, 0,
   0, 0, 0, 0, 1e6, 0,
   0, 0, 0, 0, 0, 1e-9}};

class Odom
{
public:
  Odom()
  {
    //底盘位姿及IMU角速度初始化，
    X=0.0;
    Y=0.0;
    Z=0.0;
    Yaw=0.0;
    imu_angularZ=0;
    last_time = ros::Time::now();
    pub=nh.advertise<nav_msgs::Odometry>("/odom",1000);
    sub=nh.subscribe("/feedback/cmd_vel",1000,&Odom::Callback,this);  //底盘发布的反馈速度话题
    sub_imu=nh.subscribe("/imu",1000,&Odom::Imu_Callback,this);
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
      odom_Stam.header.stamp=current_time;
      odom_Stam.header.frame_id="odom";
      odom_Stam.child_frame_id="base_footprint";
      odom_Stam.transform.translation.x=X;
      odom_Stam.transform.translation.y=Y;
      odom_Stam.transform.translation.z=Z;
      odom_Stam.transform.rotation=odom_quaternion;
      odom_Br.sendTransform(odom_Stam);  //发布广播
      loop_rate.sleep();
      ros::spinOnce();
    }
  }
  void Callback(const geometry_msgs::Twist::ConstPtr&msg);
  void Imu_Callback(const sensor_msgs::Imu::ConstPtr& msg);
protected:
  ros::Time current_time;
  ros::Time last_time;
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Subscriber sub_imu;
  ros::Publisher pub;
  geometry_msgs::Quaternion odom_quaternion;  //odom四元数
  geometry_msgs::Quaternion imu_quaternion;   //imu四元数
  geometry_msgs::TransformStamped odom_Stam;  //TF戳
  tf::TransformBroadcaster odom_Br;  //TF广播
  nav_msgs::Odometry odom;

  double imu_angularZ;
  double X,Y,Z,Roll,Pitch,Yaw;
};
void Odom::Imu_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_angularZ=msg->angular_velocity.z;
  imu_quaternion=msg->orientation;
}
void Odom::Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  current_time = ros::Time::now();
  ros::Duration dt = (current_time - last_time);
  //引入imu角速度积分得到偏航角，用于对底盘X、Y位置计算
  Yaw=Yaw+(imu_angularZ*dt.toSec());
  X=X+(msg->linear.x*cos(Yaw)*dt.toSec()-msg->linear.y*sin(Yaw)*dt.toSec());
  Y=Y+(msg->linear.x*sin(Yaw)*dt.toSec()+msg->linear.y*cos(Yaw)*dt.toSec());
  //由偏航角转化为odom的四元素
  //tf::createQuaternionMsgFromRollPitchYaw(Roll,Pitch,Yaw);
  //odom_quaternion=tf::createQuaternionMsgFromYaw(Yaw);
  odom_quaternion=imu_quaternion;
  //根据底盘是否运动，赋不同的协方差
  if(msg->linear.x!=0.0 || msg->linear.y!=0.0 ||msg->angular.z!=0.0) {
    odom.pose.covariance=ODOM_POSE_COVARIANCE;
    odom.twist.covariance=ODOM_TWIST_COVARIANCE;
  }
  else {
    odom.pose.covariance=ODOM_POSE_COVARIANCE2;
    odom.twist.covariance=ODOM_TWIST_COVARIANCE2;
  }
  odom.twist.twist.linear.x=msg->linear.x;
  odom.twist.twist.linear.y=msg->linear.y;
  odom.twist.twist.angular.z=imu_angularZ;
  odom.pose.pose.position.x=X;
  odom.pose.pose.position.y=Y;
  odom.pose.pose.position.z=Z;
  odom.pose.pose.orientation=odom_quaternion;    //使用imu角速度积分得到航向角后转化的四元数
  //odom.pose.pose.orientation=imu_quaternion;   使用imu的四元数
  odom.header.frame_id="odom";
  odom.child_frame_id="base_footprint";
  odom.header.stamp=ros::Time::now();
  last_time = current_time;
  pub.publish(odom);
}
int main(int argc,char** argv)
{
  ros::init(argc,argv,"odom");
  Odom oA;

  ros::spin();
  return 0;
}
