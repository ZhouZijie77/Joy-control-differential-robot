//
// Created by zzj on 2021/10/27.
//

#pragma once
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <arpa/inet.h>
#include <queue>
#include <chrono>
using namespace std;
struct TimeHandle {
    bool is_update;
    double elapse_time_;
    double last_timestamp;
    TimeHandle() {
        is_update = false;
        elapse_time_ = 200;
        last_timestamp = 0;
    }
};
union RL2byte {
    //short和char[2]共用内存地址，实现进制转换
    short _int;
    char _char[2];
};
namespace carnet {
    class udp_car {
    public:
        uint8_t data_to_send[13];
        TimeHandle cmd;
        RL2byte vR,vL;
    public:
        udp_car(const ros::NodeHandle &nh,
                std::string node_name);

        // ros
        ros::NodeHandle nh_;
        ros::Timer loop_timer_;
        std::string node_name_{"udp_sender_node"};

        ros::Subscriber sub;

        double vel_ratio;
        double car_width;
        // UDP 设置
        /* local address and port */
        int CarNetSocket;
        sockaddr_in addr_local;
        std::string local_ip;
        int local_port;

        /* remote address and port */
        sockaddr_in addr_remote, addr_car;
        socklen_t addr_remote_len; //// do not forget to init the 'addr_remote_len'
        std::string remote_ip;
        int remote_port;

        void joyCallback(const sensor_msgs::Joy::ConstPtr&  con);
        void timerCb();
        bool initSocket(void);
        void sendmsgs();
    };
}//namespace carnet


