#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>  
#include <cstring>
#include <string>
#include <cstdint>
#include <iostream>


class Udp 
{
public:
    Udp(int sport);
    ~Udp();

private:
    int server_port_;

    int udp_socket_;

    std::string target_ip_;
    int target_port_;

    

public:
    bool udp_send(uint8_t *pBuf, uint16_t len);
    bool udp_receive(uint8_t *pBuf, uint16_t *pLen);

    bool udp_sendto(uint8_t *pBuf, uint16_t len, std::string ip, int port);
    bool udp_receivefrom(uint8_t *pBuf, uint16_t *pLen, std::string *pIp, int *pPort);
};

