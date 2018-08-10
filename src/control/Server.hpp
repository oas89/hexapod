#include <iostream>
#include <vector>
#include <memory>
#include <stdexcept>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>


class DatagramServer {
public:
    DatagramServer(uint16_t port) {
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        server_addr.sin_port = htons(port);
        fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd < 0) throw std::runtime_error("Cannot create socket");
        int code = bind(fd, reinterpret_cast<struct sockaddr*>(&server_addr), sizeof(server_addr));
        if (code < 0) throw std::runtime_error("Cannot bind socket");
    }

    std::vector<char> receive() {
        socklen_t client_addr_length = sizeof(client_addr);
        ssize_t n = recvfrom(fd, buffer, 512, 0, reinterpret_cast<struct sockaddr*>(&client_addr), &client_addr_length);
        if (n < 0) throw std::runtime_error("Cannot receive data from socket");
        return std::vector<char>(buffer, buffer + 512);
    }

private:
    int fd;
    char buffer[512];
    sockaddr_in server_addr = {0};
    sockaddr_in client_addr = {0};
};