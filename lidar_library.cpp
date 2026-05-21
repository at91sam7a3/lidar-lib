#include "lidar_library.hpp"
#include <iostream>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <thread>

// Header bytes for packet detection
const uint8_t LIDARLibrary::HEADER[2] = {0x55, 0xAA};

LIDARLibrary::LIDARLibrary()
    : serialPort_(-1), running_(false), head_(0), tail_(0), callback_(nullptr) {
    memset(circleBuffer_, 0, sizeof(circleBuffer_));
}

LIDARLibrary::~LIDARLibrary() {
    CloseUart();
}

bool LIDARLibrary::OpenUart(const std::string& portName) {
    if (IsOpen()) {
        std::cerr << "UART port is already open" << std::endl;
        return false;
    }

    serialPort_ = ConfigureSerialPort(portName);
    if (serialPort_ < 0) {
        return false;
    }

    running_ = true;
    readThread_ = std::thread(&LIDARLibrary::ReadSerialPort, this);

    return true;
}

void LIDARLibrary::CloseUart() {
    if (!IsOpen()) {
        return;
    }

    running_ = false;

    if (readThread_.joinable()) {
        readThread_.join();
    }

    if (serialPort_ >= 0) {
        close(serialPort_);
        serialPort_ = -1;
    }
}

void LIDARLibrary::SetCallback(PacketCallback callback) {
    callback_ = callback;
}

bool LIDARLibrary::IsOpen() const {
    return serialPort_ >= 0 && running_;
}

int LIDARLibrary::ConfigureSerialPort(const std::string& portName) {
    int fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening serial port: " << strerror(errno) << std::endl;
        return -1;
    }

    termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    // Set baud rate to 230400 (fixed)
    cfsetospeed(&tty, B230400);
    cfsetispeed(&tty, B230400);

    // Set character size to 8 bits
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;

    // Disable break processing
    tty.c_iflag &= ~IGNBRK;

    // Disable local flags
    tty.c_lflag = 0;

    // Disable output flags
    tty.c_oflag = 0;

    // Set minimum characters to read and timeout
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;

    // Disable software flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Enable receiver and local mode
    tty.c_cflag |= (CLOCAL | CREAD);

    // Disable parity
    tty.c_cflag &= ~(PARENB | PARODD);

    // Use one stop bit
    tty.c_cflag &= ~CSTOPB;

    // Disable hardware flow control
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    return fd;
}

void LIDARLibrary::ReadSerialPort() {
    uint8_t buf[256];

    while (running_) {
        int n = read(serialPort_, buf, sizeof(buf));
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                AddToBuffer(buf[i]);
            }
            ProcessBuffer();
        } else if (n < 0) {
            std::cerr << "Error reading from UART: " << strerror(errno) << std::endl;
            running_ = false;
            break;
        }
    }
}

void LIDARLibrary::AddToBuffer(uint8_t data) {
    circleBuffer_[head_] = data;
    head_ = (head_ + 1) % BUFFER_SIZE;
    if (head_ == tail_) {
        tail_ = (tail_ + 1) % BUFFER_SIZE;
        std::cerr << "Buffer overflow detected" << std::endl;
    }
}

uint16_t LIDARLibrary::Available() const {
    return (BUFFER_SIZE + head_ - tail_) % BUFFER_SIZE;
}

uint16_t LIDARLibrary::FreeSpace() const {
    return (BUFFER_SIZE - 1 - head_ + tail_) % BUFFER_SIZE;
}

uint16_t LIDARLibrary::RemoveBuffer(uint16_t size) {
    uint16_t removed_count = 0;
    for (uint16_t i = 0; i < size && Available() > 0; i++) {
        tail_ = (tail_ + 1) % BUFFER_SIZE;
        removed_count++;
    }
    return removed_count;
}

uint16_t LIDARLibrary::PeekBuffer(uint8_t* data, uint16_t size) {
    uint16_t peeked_count = 0;
    for (uint16_t i = 0; i < size && Available() > 0; i++) {
        data[i] = circleBuffer_[(tail_ + i) % BUFFER_SIZE];
        peeked_count++;
    }
    return peeked_count;
}

uint16_t LIDARLibrary::ReadFromBuffer(uint8_t* data, uint16_t size) {
    uint16_t read_count = 0;
    for (uint16_t i = 0; i < size && Available() > 0; i++) {
        data[i] = circleBuffer_[tail_];
        tail_ = (tail_ + 1) % BUFFER_SIZE;
        read_count++;
    }
    return read_count;
}

bool LIDARLibrary::FindHeader() {
    while (true) {
        if (Available() < 2) {
            return false;
        }

        uint8_t data[2];
        uint16_t peeked = PeekBuffer(data, 2);
        if (peeked == 2) {
            if (data[0] == HEADER[0] && data[1] == HEADER[1]) {
                return true;
            } else {
                RemoveBuffer(1);
            }
        }
    }
}

LidarData LIDARLibrary::ParseData(uint8_t* data, uint16_t size) {
    LidarData lidarData{};

    if (size < 60) {
        std::cerr << "Insufficient data for parsing" << std::endl;
        return lidarData;
    }

    uint8_t type = data[2];
    uint8_t data_size = data[3];

    if (type == 0x23) {
        uint16_t speed = (data[5] << 8) | data[4];
        uint16_t start_angle = (((data[7] & 0x7F) << 8) + data[6]) - 0x2000;
        uint16_t end_angle = (((data[57] & 0x7F) << 8) | data[56]) - 0x2000;
        uint16_t crc = (data[59] << 8) | data[58];

        double start_angle_deg = start_angle / 64.0;
        double end_angle_deg = end_angle / 64.0;

        double dif = end_angle_deg - start_angle_deg;
        if (end_angle_deg < start_angle_deg) {
            dif = 360.0 - start_angle_deg + end_angle_deg;
        }

        lidarData.start_angle = start_angle_deg;
        lidarData.step_size = dif / (data_size - 1);

        for (int i = 0; i < data_size; i++) {
            uint16_t offset = i * 3;
            uint16_t distance = (((data[9 + offset] & 0x3F) << 8) | data[8 + offset]) * 0.1;
            uint8_t xdata = data[9 + offset] >> 6;
            uint8_t strength = data[10 + offset];

            double sample_angle = start_angle_deg + (dif / (data_size - 1)) * i;
            if (sample_angle >= 360.0) {
                sample_angle -= 360.0;
            }

            lidarData.distances[i] = distance;
            if (strength == 0) {
                lidarData.distances[i] = 0;
            }
            lidarData.xdata[i] = xdata;
            lidarData.intensities[i] = strength;
        }
    }

    return lidarData;
}

void LIDARLibrary::ProcessBuffer() {
    while (Available() >= 60) {
        if (!FindHeader()) {
            break;
        }

        if (Available() < 60) {
            break;
        }

        uint8_t data[60];
        uint16_t read_count = ReadFromBuffer(data, 60);

        if (read_count == 60) {
            LidarData lidarData = ParseData(data, read_count);
            if (callback_) {
                callback_(lidarData);
            }
        }
    }
}

