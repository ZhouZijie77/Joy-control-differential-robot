#pragma once
#include <netinet/in.h>
#include <sys/socket.h>
#include <string>

union RL2byte {
  short _int;
  uint8_t _byte[2];
};
class UDPSender {
 public:
  uint8_t data_to_send[13];

  socklen_t addr_remote_len;

 public:
  UDPSender(std::string target_ip, int target_port, std::string host_ip,
            int host_port);

  void setVel(short vL, short vR);
  void printVel();

  void sendMsgs();

 private:
  const double max_vel;
  int CarNetSocket;
  sockaddr_in addr_local;
  sockaddr_in addr_remote;
  RL2byte vR, vL;
  std::string target_ip;
  int target_port;
  std::string host_ip;
  int host_port;

 private:
  bool initSocket();
};
