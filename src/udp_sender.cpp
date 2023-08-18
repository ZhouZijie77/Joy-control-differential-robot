#include "udp_sender.h"
#include <arpa/inet.h>
#include <cstring>
#include <iostream>
UDPSender::UDPSender(std::string target_ip, int target_port,
                     std::string host_ip, int host_port)
    : max_vel(100) {
  memset(this->data_to_send, 0, sizeof(this->data_to_send));
  data_to_send[0] = 0x28;  // 帧信息
  data_to_send[1] = 0x00;
  data_to_send[2] = 0x00;
  data_to_send[3] = 0x06;
  data_to_send[4] = 0x01;
  data_to_send[5] = 0x23;
  data_to_send[6] = 0x00;
  data_to_send[7] = 0x20;
  this->target_ip = target_ip;
  this->target_port = target_port;
  this->host_ip = host_ip;
  this->host_port = host_port;

  vL._int = vR._int = 0;

  if (!initSocket()) {
    std::cerr << "init socket failed!" << std::endl;
    abort();
  }

  std::cout << "max vel send to car is 100(0.16m/s)" << std::endl;
}

void UDPSender::setVel(short vL, short vR) {
  vL = vL > max_vel ? max_vel : vL;
  vR = vR > max_vel ? max_vel : vR;
  this->vL._int = vL;
  this->vR._int = vR;
  // std::cout<<"set vL="<<this->vL._int<<" "<<"vR="<<this->vR._int<<std::endl;
}

void UDPSender::printVel() {
  std::cout << "vL=" << vL._int << "vR=" << vR._int << std::endl;
}

void UDPSender::sendMsgs() {
  // counter作用在于连续发两帧控制左右履带
  static int counter = 0;
  counter == 1 ? counter = 0 : counter = 1;
  printf("vL = %d  vR=%d\n", vL._int, vR._int);

  if (counter == 0) {
    data_to_send[8] = 0x01;
    data_to_send[9] = vL._byte[0];
    data_to_send[10] = vL._byte[1];
  } else {
    data_to_send[8] = 0x02;
    data_to_send[9] = vR._byte[0];
    data_to_send[10] = vR._byte[1];
  }
  sendto(CarNetSocket, data_to_send, sizeof(data_to_send), 0,
         (struct sockaddr *)&addr_remote, addr_remote_len);
}

bool UDPSender::initSocket() {
  CarNetSocket =
      socket(AF_INET, SOCK_DGRAM, 0);  // 创建socket文件描述符，是一个非负整数
  // AF_INET表示采用IPv4协议
  // SOCK_DGRAM表示套接字类型为UDP数据报
  if (CarNetSocket < 0) {
    std::cerr << "creating CarNetSocket failed!" << std::endl;
    return false;
  } else {
    std::cout << "creating CarNetSocket succeed!" << std::endl;
  }
  // 设置目标ip、端口
  memset(&addr_remote, 0, sizeof(addr_remote));
  addr_remote.sin_addr.s_addr = inet_addr(target_ip.c_str());
  addr_remote.sin_family = AF_INET;
  addr_remote.sin_port = htons(target_port);
  addr_remote_len = sizeof(this->addr_remote);

  std::cout << "Target IP: " << target_ip << std::endl;
  std::cout << "Target Port: " << target_port << std::endl;
  return true;
}
