#include "udp.h"

Udp::Udp(int sport)
{
    server_port_    = sport;

    //socket
    udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_ < 0) 
    {
        perror("socket creation failed");
        return;
    }

    // 设置为非阻塞模式
    int flags = fcntl(udp_socket_, F_GETFL, 0);
    fcntl(udp_socket_, F_SETFL, flags | O_NONBLOCK);

    // 设置服务器地址
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port_);
    server_addr.sin_addr.s_addr = INADDR_ANY;  // 修改为INADDR_ANY接收广播

    // 绑定服务器地址
    if (bind(udp_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) 
    {
        perror("bind failed");
        close(udp_socket_);
        udp_socket_ = -1;
    }
}

Udp::~Udp()
{
    if (udp_socket_ >= 0) 
    {
        close(udp_socket_);
    }
}

bool Udp::udp_send(uint8_t *pBuf, uint16_t len)
{
    if (udp_socket_ < 0) 
        return false;

    struct sockaddr_in target_addr;
    memset(&target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(target_port_);
    target_addr.sin_addr.s_addr = inet_addr(target_ip_.c_str());
    
    ssize_t sent = sendto(udp_socket_, pBuf, len, 0, 
                         (struct sockaddr*)&target_addr, sizeof(target_addr));

    return sent == len;
}

bool Udp::udp_receive(uint8_t *pBuf, uint16_t *pLen)
{
    if (udp_socket_ < 0) 
        return false;

    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    
    ssize_t received = recvfrom(udp_socket_, pBuf, 1024, 0, 
                               (struct sockaddr*)&client_addr, &addr_len);
    if (received > 0) 
    {
        *pLen = received;
        target_ip_ = inet_ntoa(client_addr.sin_addr);
        target_port_ = ntohs(client_addr.sin_port);

        return true;
    }

    return false;
}

bool Udp::udp_sendto(uint8_t *pBuf, uint16_t len, std::string ip, int port)
{
    if (udp_socket_ < 0) 
        return false;
    
    struct sockaddr_in target_addr;
    memset(&target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(port);
    target_addr.sin_addr.s_addr = inet_addr(ip.c_str());
    
    ssize_t sent = sendto(udp_socket_, pBuf, len, 0, 
                         (struct sockaddr*)&target_addr, sizeof(target_addr));

    return sent == len;
}

bool Udp::udp_receivefrom(uint8_t *pBuf, uint16_t *pLen, std::string *pIp, int *pPort)
{
    if (udp_socket_ < 0) 
        return false;
    
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    
    ssize_t received = recvfrom(udp_socket_, pBuf, 1024, 0, 
                               (struct sockaddr*)&client_addr, &addr_len);
    if (received > 0) 
    {
        *pLen = received;
        *pIp = inet_ntoa(client_addr.sin_addr);
        *pPort = ntohs(client_addr.sin_port);
        return true;
    }

    return false;
}
