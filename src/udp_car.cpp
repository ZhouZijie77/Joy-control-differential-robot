#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <udp_sender.h>
class Udp_car {
 private:
  ros::NodeHandle nh;
  ros::Subscriber joy_sub;
  ros::Timer loop_timer;
  double vel_ratio;
  double car_width;
  std::string host_ip;
  std::string target_ip;
  int host_port;
  int target_port;
  UDPSender* sender;

 public:
  Udp_car() : nh("~") {
    nh.param<double>("vel_ratio", vel_ratio, 100.0);
    nh.param<double>("car_width", car_width, 1.2);
    nh.param<std::string>("host_ip", host_ip, "192.168.1.102");
    nh.param<int>("host_port", host_port, 8001);
    nh.param<std::string>("target_ip", target_ip, "192.168.1.10");
    nh.param<int>("target_ip", target_port, 4001);
    sender = new UDPSender(target_ip, target_port, host_ip, host_port);
    joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 100, &Udp_car::joyCallback, this);
    loop_timer = nh.createTimer(ros::Duration(0.02), boost::bind(&Udp_car::timerCb, this));
  }

  void joyCallback(const sensor_msgs::Joy::ConstPtr& con) {
    float vel_linear = con->axes[4];  // 用手柄右摇杆上下即axes[4]来控制线速度
    float vel_angular = con->axes[3];  // 用手柄右摇杆左右即axes[3]来控制角速度
    // 差速转向，通过质心线速度和角速度求两侧履带速度，受底盘控制器限制速度范围为(-1000,1000)
    short vL = (short)((-vel_linear - vel_angular * car_width / 2) * vel_ratio);
    short vR = (short)((-vel_linear + vel_angular * car_width / 2) * vel_ratio);
    sender->setVel(vL, vR);
  }
  void timerCb() { sender->sendMsgs(); }
  ~Udp_car() { delete sender; }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "udp_car");
  Udp_car udp_car;
  ros::spin();
  return 0;
}