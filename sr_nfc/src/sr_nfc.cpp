/*
 *函数解释
 nfc_init();    初始化libnfc,必须在调用任何其他libnfc函数之前调用此函数
 nfc_exit();    去初始化libnfc，应在关闭所有打开的设备之后和应用程序终止之前调用
 nfc_open();    打开nfc设备
 nfc_close();   关闭nfc设备
 nfc_perror();  显示nfc设备上发生的最后一个错误
*/
#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "msg/sr_sensor.h"
#include "nfc/nfc.h"
#include "signal.h"
#include "sr_nfc.h"
#define MAX_FRAME_LEN 264
#define MAX_DEVICE_COUNT 2
#define NUM 20
static nfc_device *pnd;
static nfc_context *context;

using namespace std;
class NfcRos
{
public:
  NfcRos();
  ~NfcRos();
  void ESP32Callback(const msg::sr_sensor::ConstPtr &msg);
  static void stop_dep_communication(int sig);
  void PUBGoal();
private:
  ros::NodeHandle node;  //ros句柄
  move_base_msgs::MoveBaseActionGoal Goal[NUM];
  ros::Subscriber sub_ESP32;
  ros::Publisher pub_goal;
  string RouteName;
  uint16_t TempCheck=0;
  int Target_Point_Number;
  //static nfc_device *pnd;
  //static nfc_context *context;
  uint8_t  abtRx[MAX_FRAME_LEN];
  nfc_target nt;
  int  szRx;


};
NfcRos::NfcRos()
{
  uint8_t  abtTx[] = "I LOVE CC EVERYDAY!";
  //加载路线点位数量
  node.getParam("/Target_Point_Number",Target_Point_Number);
  node.param<string>("/sr_nfc_node/RouteName",RouteName,"A");
  pub_goal=node.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal",100);
  sub_ESP32=node.subscribe("other_sensors/data",100,&NfcRos::ESP32Callback,this);
  cout<<"RouteName==================="<<RouteName<<endl;
  for (int i=0;i<=Target_Point_Number;i++) {
    Goal[i].goal.target_pose.header.frame_id="map";
    node.getParam(RouteName+"/positionX_"+to_string(i),Goal[i].goal.target_pose.pose.position.x);
    node.getParam(RouteName+"/positionY_"+to_string(i),Goal[i].goal.target_pose.pose.position.y);
    node.getParam(RouteName+"/positionZ_"+to_string(i),Goal[i].goal.target_pose.pose.position.z);
    node.getParam(RouteName+"/orientationX_"+to_string(i),Goal[i].goal.target_pose.pose.orientation.x);
    node.getParam(RouteName+"/orientationY_"+to_string(i),Goal[i].goal.target_pose.pose.orientation.y);
    node.getParam(RouteName+"/orientationZ_"+to_string(i),Goal[i].goal.target_pose.pose.orientation.z);
    node.getParam(RouteName+"/orientationW_"+to_string(i),Goal[i].goal.target_pose.pose.orientation.w);
    printf("X=%f\n",Goal[i].goal.target_pose.pose.position.x);
    printf("Y=%f\n",Goal[i].goal.target_pose.pose.position.y);
    printf("Z=%f\n",Goal[i].goal.target_pose.pose.position.z);
    printf("x=%f\n",Goal[i].goal.target_pose.pose.orientation.x);
    printf("y=%f\n",Goal[i].goal.target_pose.pose.orientation.y);
    printf("z=%f\n",Goal[i].goal.target_pose.pose.orientation.z);
    printf("w=%f\n",Goal[i].goal.target_pose.pose.orientation.w);
  }

  nfc_init(&context);
  if (context == NULL) {
    cout<<"无法初始化libnfc"<<endl;
    return ;
  }
  cout<<"初始化libnfc成功"<<endl;
  nfc_connstring connstrings[MAX_DEVICE_COUNT];
  size_t szDeviceFound = nfc_list_devices(context, connstrings, MAX_DEVICE_COUNT);
  if (szDeviceFound == 1) {
      pnd = nfc_open(context, connstrings[0]);
    } else if (szDeviceFound > 1) {
      pnd = nfc_open(context, connstrings[1]);
    } else {
      cout<<"未发现NFC设备"<<endl;
      nfc_exit(context);
      return ;
    }
  nt.nm.nmt=NMT_DEP;
  nt.nm.nbr=NBR_UNDEFINED;
  nt.nti.ndi.abtNFCID3[0]=0x12;
  nt.nti.ndi.abtNFCID3[1]=0x34;
  nt.nti.ndi.abtNFCID3[2]=0x56;
  nt.nti.ndi.abtNFCID3[3]=0x78;
  nt.nti.ndi.abtNFCID3[4]=0x9a;
  nt.nti.ndi.abtNFCID3[5]=0xbc;
  nt.nti.ndi.abtNFCID3[6]=0xde;
  nt.nti.ndi.abtNFCID3[7]=0xff;
  nt.nti.ndi.abtNFCID3[8]=0x00;
  nt.nti.ndi.abtNFCID3[9]=0x00;
  nt.nti.ndi.szGB=4;
  nt.nti.ndi.abtGB[0] = {0x12};
  nt.nti.ndi.abtGB[1] = {0x34};
  nt.nti.ndi.abtGB[2] = {0x56};
  nt.nti.ndi.abtGB[3] = {0x78};
  nt.nti.ndi.ndm=NDM_UNDEFINED;
  nt.nti.ndi.btDID = 0x00;
  nt.nti.ndi.btBS = 0x00;
  nt.nti.ndi.btBR = 0x00;
  nt.nti.ndi.btTO = 0x00;
  nt.nti.ndi.btPP = 0x01;
  if (pnd == NULL) {
    printf("Unable to open NFC device.\n");
    nfc_exit(context);
    return ;
  }
  printf("NFC device: %s opened\n", nfc_device_get_name(pnd));
  //signal(SIGINT, stop_dep_communication);  //

  PUBGoal();
}
//*********循环接收目标点编号，并发布goal**********//
void NfcRos::PUBGoal()
{
  ros::Rate loop(1);
  while(ros::ok())
  {
    printf("Waiting for initiator request...\n");  //正在等待发起器请求
    //将NFC设备初始化为target
    if (( szRx = nfc_target_init(pnd, &nt,abtRx, sizeof(abtRx), 500)) < 0) {
      nfc_perror(pnd, "nfc_target_init");
      loop.sleep();
      ros::spinOnce();
      continue ;
    }

    //接收字节函数
    printf("Initiator request received. Waiting for data...\n");
    if ((szRx = nfc_target_receive_bytes(pnd, abtRx, sizeof(abtRx), 0)) < 0) {
      nfc_perror(pnd, "nfc_target_receive_bytes");
      continue ;
    }
    abtRx[(size_t) szRx] = '\0';
    int num;
    printf("num=%d,Received: %s\n",num=sizeof (abtRx),abtRx);

    //解析数据
    for (int i=0;i<(size_t)szRx;i++) {
      printf("abtRx[%d]=%c\n",i,abtRx[i]);
      TXNfcData.data[0]=(abtRx[i]-48)*10+(abtRx[i+1]-48);
      i++;
    }
    printf("TXNfcData.data[0]=%d\n",TXNfcData.data[0]);


    //printf("RXNfcData: %x,%x,%x,%x\n", RXNfcData.data[0],RXNfcData.data[1],RXNfcData.data[2],RXNfcData.data[3]);
    //发送字节函数
    if (nfc_target_send_bytes(pnd, TXNfcData.data, sizeof(TXNfcData.data), 0) < 0) {
      nfc_perror(pnd, "nfc_target_send_bytes");
      continue ;
    }


//    TempCheck=0;
//    for(u8 i=0;i<sizeof(RXNfcData.data)-2;i++)
//    {
//       TempCheck += RXNfcData.data[i];
//    }
//    //头和校验正确
//    if(RXNfcData.prot.Header == HEADER && RXNfcData.prot.Check == TempCheck) {
//      //消息赋值
    for (int i=0;i<NUM;i++) {
      if(i==TXNfcData.data[0]) {
        Goal[i].goal.target_pose.header.stamp=ros::Time::now();

        pub_goal.publish(Goal[i]);
        //memset(RXNfcData.data,0,sizeof(RXNfcData.data));
        break;
      }
    }
//    }
//    else {
//      cout<<"接收数据校验错误"<<endl;
//    }
    loop.sleep();
    ros::spinOnce();
  }
}
//*********消毒控制版ESP32数据处理回调函数*********//
void NfcRos::ESP32Callback(const msg::sr_sensor::ConstPtr &msg)
{
  TXNfcData.prot.Header  = HEADER;
  TXNfcData.prot.Voltage = msg->sensor_data[3];
  //printf("msg->sensor_data[3]=%d\n",msg->sensor_data[3]);
  TXNfcData.prot.Retain  = 0;
  TXNfcData.prot.Check=0;
  for(uint8_t i=0;i < sizeof(TXNfcData.data) - 2;i++)
  {
    TXNfcData.prot.Check += TXNfcData.data[i];
  }
  printf("TXNfcData: %x,%x,%x,%x\n", TXNfcData.data[0],TXNfcData.data[1],TXNfcData.data[2],TXNfcData.data[3]);

}
////*********析构函数*********//
NfcRos::~NfcRos()
{
  nfc_close(pnd);     //关闭nfc设备
  nfc_exit(context);  //去初始化libnfc，应在关闭所有打开的设备之后和应用程序终止之前调用。
}

static void stop_dep_communication(int sig)
{
//  (void) sig;
//  if (pnd != NULL) {
//    nfc_abort_command(pnd);
//  } else {
//    nfc_exit(context);
//    exit(EXIT_FAILURE);
//  }
}
int main(int argc, char *argv[])
{
  ros::init(argc,argv,"sr_nfc");

  int  szRx;
  NfcRos nfcros;
  ros::spin();
  return 0;
}
