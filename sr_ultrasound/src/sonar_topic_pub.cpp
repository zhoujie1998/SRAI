#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <msg/ultrasonic.h>
#include "math.h"
typedef struct
{
    float lastP;		//上次的协方差
    float nowP;			//本次的协方差
    float x_hat;		//卡尔曼滤波的计算值，即为后验最优值
    float Kg;			  //卡尔曼增益系数
    float Q;			  //过程噪声
    float R;			  //测量噪声
}Kalman;
using namespace std;
std::vector<sensor_msgs::Range> sonar_msgs;
std::vector<ros::Publisher> pub_sonar_topic;
std::vector<Kalman> KF;
bool use_kalman;

//卡尔曼滤波器
float Kalman_Filter(Kalman *KF, float input)
{
  float output = 0, x_t;						      //output为卡尔曼滤波计算值
  x_t = KF->x_hat;							          //当前先验预测值 = 上一次最优值   对于动态环境,添加速度参数还可提升滤波效果
  KF->nowP = KF->lastP + KF->Q;				    //本次的协方差矩阵
  KF->Kg = KF->nowP / (KF->nowP + KF->R);	//卡尔曼增益系数计算
  //对于斜面等无反射回波的数据处理为上一次的最优
//  if(input == 0) {
//    output = x_t;
//    return output;
//  }
  output = x_t + KF->Kg*(input - x_t); 		//当前最优值
  KF->x_hat = output;							        //更新最优值
  KF->lastP = (1 - KF->Kg) * KF->nowP;		//更新协方差矩阵
  return output;
}


void UltrasonicCallback(const msg::ultrasonic::ConstPtr &msg)
{
  //当声纳层消息数量不等于超声波原始数据数量
  if(sonar_msgs.size() != msg->data.size()  ) {
    sonar_msgs.resize(msg->data.size());
    ROS_ERROR_STREAM("ultrasound_num ,The param not equal to ultrasonic.data.size");
    ROS_ERROR_STREAM("Please set ultrasound_num = " << msg->data.size() );
    return ;
  }
  for (int i=0; i<msg->data.size(); i++)
  {
    //转换单位mm->m
    sonar_msgs[i].range = msg->data[i]*0.001;
    if(use_kalman ==true) sonar_msgs[i].range = Kalman_Filter(&KF[i],sonar_msgs[i].range);
    //当超声波无数据时返回或大于max_range ，赋值为max_range,     rang_sensor_layer才能清除
    if(sonar_msgs[i].range == 0 || sonar_msgs[i].range > sonar_msgs[i].max_range)  
      sonar_msgs[i].range = sonar_msgs[i].max_range;
    sonar_msgs[i].header.stamp=ros::Time::now();
    pub_sonar_topic[i].publish(sonar_msgs[i]);
  }
}

int main(int argc, char *argv[])
{
  int ultrasound_num;
  string pub_topic_sonar,frame_id;
  double min_range,max_range,field_of_view,Q,R;
  ros::init(argc,argv,"sonar_topic_pub");
  ros::NodeHandle n("~");
  //加载参数
  n.param<int>("ultrasound_num", ultrasound_num, 4);
   n.param<bool>("use_kalman", use_kalman, false);
  n.param<string>("pub_topic_sonar", pub_topic_sonar, "/sonar");
	n.param<string>("frame_id", frame_id, "/sonar");
	n.param<double>("min_range", min_range, 0.1);
	n.param<double>("max_range", max_range, 0.5);
	n.param<double>("field_of_view", field_of_view, 0.8);

	
  sonar_msgs.resize(ultrasound_num);
  pub_sonar_topic.resize(ultrasound_num);
  KF.resize(ultrasound_num);
  for (int i=0; i<ultrasound_num; i++) {
    //消息数组初始化
    sonar_msgs[i].header.frame_id= frame_id+to_string(i);
    sonar_msgs[i].field_of_view = field_of_view;
    sonar_msgs[i].radiation_type = 0; //超声波0 ，红外线1
    sonar_msgs[i].min_range = min_range;
    sonar_msgs[i].max_range = max_range;
    //初始化卡尔曼
    KF[i].Q = Q;
    KF[i].R = R;
    KF[i].Kg = 0;
    KF[i].lastP = 1;
    KF[i].x_hat = 0;
    //注册话题发布者
    pub_sonar_topic[i] =n.advertise<sensor_msgs::Range>(pub_topic_sonar+to_string(i),100);
  }
  ros::Subscriber sub_ultrasound=n.subscribe<msg::ultrasonic>("/ultrasonic/data",100,UltrasonicCallback);

  ros::spin();
  return 0;
}
