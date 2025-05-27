#include <linux/serial.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <memory>
#include <string>
#include <vector>
#include <termios.h>

class Sender {
public:
    Sender(std::string device, int baudrate);
    ~Sender();
    bool send(const std::vector<uint8_t>& data, size_t length);
    bool isOpen() const;
    void open();
    void close();

private:
    std::string device_;
    int baudrate_;
    int fd_;
    struct termios options_;

    bool setBaudRate(int baudrate);
    bool configurePort();
    void handleError(const std::string& message);

};

/*
author: Yamato Kamei
date: 2025-05-27
*/