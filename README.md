# 一、功能包介绍

用罗技F710无线游戏手柄+ROS Melodic控制差速底盘。

## 工作过程：

* 接收手柄发送的topic
* 计算两侧履带速度
* 给cannet发送udp报文
* cannet将udp报文解析成can报文发送给底盘

# 二、硬件连接介绍

手柄通过无线接收器与电脑相连，电脑通过网线或者无线路由器与cannet连接。cannet与底盘电机相连。

# 三、操作流程

## 1、 安装joy与joystick_driver包

`sudo apt-get install ros-melodic-joy`

`sudo apt-get install ros-melodic-joystick-drivers`

## 2、测试手柄

将手柄的无线信号接收器插到电脑上执行

`ls /dev/input`

应该能够看到手柄的输入接口js0

下面进行手柄话题发布的检测

先使用`roscore`命令激活Master管理器，之后执行

`rosrun joy joy_node`

新打开一个终端输入监控话题

`rostopic echo /joy`

应该能够看到手柄按钮和遥感每次被操作都会更新数据。根据不同操作按键数据更新确定每个按键对应的axes和button位数。

根据自己的控制习惯修改udp_car/src/udp_car.cpp的joyCallback()回调函数中修改。

## 4、修改ip地址

查看自己本地的ip地址，然后在udp_car/src/udp_car.cpp文件中修改本机ip地址

注意，有线连接和wifi连接的本机地址可能会有差异。

## 3、编译启动

回到ros工作主目录下

`catkin_make`

`source devel/setup.bash`

`roslaunch udp_car udp_car.launch`

操作手柄驱动履带车！







