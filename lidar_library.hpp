#ifndef LIDAR_LIBRARY_HPP
#define LIDAR_LIBRARY_HPP

#include <functional>
#include <cstdint>
#include <atomic>
#include <thread>
#include <string>

// Forward declaration of LidarData struct
struct LidarData
{
    double start_angle_deg;
    double step_size_deg;
    double distances[16];
    double intensities[16];
    uint8_t xdata[16];
};

// LIDAR Library class for handling serial communication with LIDAR device
class LIDARLibrary {
public:
    // Callback function type for when a complete packet is parsed
    using PacketCallback = std::function<void(const LidarData&)>;

    LIDARLibrary();
    ~LIDARLibrary();

    // Open UART port and start reading thread (baudrate fixed at 230400)
    // Returns true on success, false on failure
    bool OpenUart(const std::string& portName);

    // Close UART port and stop reading thread
    void CloseUart();

    // Set callback function to be called when a packet is parsed
    void SetCallback(PacketCallback callback);

    // Check if UART is currently open
    bool IsOpen() const;

private:
    // UART file descriptor
    int serialPort_;

    // Reading thread
    std::thread readThread_;
    std::atomic<bool> running_;

    // Circular buffer for UART data
    static const uint16_t BUFFER_SIZE = 1024;
    uint8_t circleBuffer_[BUFFER_SIZE];
    uint16_t head_;
    uint16_t tail_;

    // Callback function
    PacketCallback callback_;

    // Thread function for reading UART data
    void ReadSerialPort();

    // UART configuration helper
    int ConfigureSerialPort(const std::string& portName);

    // Buffer management methods
    void AddToBuffer(uint8_t data);
    uint16_t Available() const;
    uint16_t FreeSpace() const;
    uint16_t RemoveBuffer(uint16_t size);
    uint16_t PeekBuffer(uint8_t* data, uint16_t size);
    uint16_t ReadFromBuffer(uint8_t* data, uint16_t size);

    // Packet parsing methods
    bool FindHeader();
    LidarData ParseData(uint8_t* data, uint16_t size);
    void ProcessBuffer();

    // Header for packet detection
    static const uint8_t HEADER[2];
};

#endif // LIDAR_LIBRARY_HPP

