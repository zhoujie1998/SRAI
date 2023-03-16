功能简介：
本功能包为任务仿真，模拟新建任务后，机器人导航。
在任务进行中，当电量不足，机器人会自动去充电桩，App端无需发布话题/GoHome
在任务进行中，当消毒液不足，机器人会自动去消毒站，App端无需发布话题/GoDisinfectionStation，并且在补充消毒液后可以继续任务

编译：
将该功能包放置在目录/home/srai/srbot_ws/src/
编译前先单独编译srv和msg这两个功能包
cd ~/srbot_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="msg"
catkin_make -DCATKIN_WHITELIST_PACKAGES="srv"
再编译test_nevigation
catkin_make -DCATKIN_WHITELIST_PACKAGES="test_nevigation"

使用：
roslaunch test_navigation start.launch


运行可能需要安装相关仿真工具：Gazbo、Rviz
sudo apt-get install ros-kinetic-rviz
sudo apt-get install ros-kinetic-gazbo-ros


