#ifndef NETWORK_CONFIG_H
#define NETWORK_CONFIG_H


typedef struct{
  String ssid;
  String password;
  IPAddress microros_agent_ip_address;
  uint32_t microros_agent_port;
}NETWORK_CONFIG;

bool configureNetwork(bool forceConfigure, NETWORK_CONFIG *networkConfig);

#endif