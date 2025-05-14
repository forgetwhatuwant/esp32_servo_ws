// servo_serial_bridge_cpp.cpp
#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <boost/asio.hpp>
#include <sstream>
#include <iomanip>
#include <string>
#include <thread>
#include <mutex>
#include <chrono>
#include <deque>

// Debug flag
#define DEBUG true

// Protocol constants
#define START_BYTE 0xAA
#define END_BYTE 0x55
#define CMD_SET_ANGLE 0x01
#define CMD_HEARTBEAT 0x02
#define RESP_STATE 0x03
#define RESP_ERROR 0x04

// Error codes
#define ERR_NONE 0x00
#define ERR_INVALID_ANGLE 0x01
#define ERR_SERVO_FAILURE 0x02

// Ring buffer size
#define RING_BUFFER_SIZE 1024

// Simple message structure
struct SimpleMessage {
    uint8_t start;
    uint8_t type;
    uint8_t data;
    uint8_t end;
};

class ServoSerialBridge {
private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_;
    ros::Publisher state_pub_;
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    std::thread read_thread_;
    std::mutex serial_mutex_;
    bool running_;
    std::string port_;
    int baud_rate_;
    std::chrono::steady_clock::time_point last_heartbeat_;
    const std::chrono::seconds HEARTBEAT_TIMEOUT{2};
    const std::chrono::seconds RECONNECT_INTERVAL{1};
    
    // Ring buffer for incoming data
    std::deque<uint8_t> rx_buffer_;
    std::mutex buffer_mutex_;

    bool openSerialPort() {
        try {
            serial_.open(port_);
            serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
            serial_.set_option(boost::asio::serial_port_base::character_size(8));
            serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
            
            // Wait for serial port to be ready
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            if (DEBUG) {
                ROS_INFO("Serial port opened successfully on %s", port_.c_str());
            }
            return true;
        } catch (const boost::system::system_error& e) {
            ROS_ERROR("Failed to open serial port: %s", e.what());
            return false;
        }
    }

    void sendMessage(uint8_t type, uint8_t data) {
        try {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            if (!serial_.is_open()) {
                ROS_ERROR("Cannot send message: serial port is not open");
                return;
            }

            SimpleMessage msg;
            msg.start = START_BYTE;
            msg.type = type;
            msg.data = data;
            msg.end = END_BYTE;
            
            // Debug print
            if (DEBUG) {
                ROS_INFO("Sending - Type: 0x%02X, Data: 0x%02X", type, data);
            }
            
            // Send message
            boost::asio::write(serial_, boost::asio::buffer(&msg, sizeof(SimpleMessage)));
        } catch (const boost::system::system_error& e) {
            ROS_ERROR("Failed to write to serial port: %s", e.what());
            handleSerialError();
        }
    }

    void handleSerialError() {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_.is_open()) {
            try {
                serial_.close();
            } catch (...) {
                // Ignore errors during close
            }
        }
    }

    void readSerial() {
        std::vector<uint8_t> temp_buffer;
        temp_buffer.reserve(1024);
        
        while (running_) {
            try {
                if (!serial_.is_open()) {
                    if (!openSerialPort()) {
                        std::this_thread::sleep_for(RECONNECT_INTERVAL);
                        continue;
                    }
                }

                // Read available data
                temp_buffer.resize(1024);  // Use a fixed buffer size
                size_t len = serial_.read_some(boost::asio::buffer(temp_buffer));
                
                if (len > 0) {
                    // Add to ring buffer
                    {
                        std::lock_guard<std::mutex> lock(buffer_mutex_);
                        rx_buffer_.insert(rx_buffer_.end(), temp_buffer.begin(), temp_buffer.begin() + len);
                        if (rx_buffer_.size() > RING_BUFFER_SIZE) {
                            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + (rx_buffer_.size() - RING_BUFFER_SIZE));
                        }
                    }
                    
                    // Process messages from buffer
                    processBuffer();
                }

                // Small sleep to prevent CPU hogging
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } catch (const boost::system::system_error& e) {
                ROS_ERROR("Serial read error: %s", e.what());
                handleSerialError();
                std::this_thread::sleep_for(RECONNECT_INTERVAL);
            }
        }
    }

    void processBuffer() {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        
        while (rx_buffer_.size() >= sizeof(SimpleMessage)) {
            // Look for start byte
            while (!rx_buffer_.empty() && rx_buffer_.front() != START_BYTE) {
                rx_buffer_.pop_front();
            }
            
            if (rx_buffer_.size() < sizeof(SimpleMessage)) {
                break;
            }
            
            // Check if we have a complete message
            SimpleMessage msg;
            std::copy_n(rx_buffer_.begin(), sizeof(SimpleMessage), reinterpret_cast<uint8_t*>(&msg));
            
            if (msg.end == END_BYTE) {
                // Debug print message details
                if (DEBUG) {
                    ROS_INFO("Received - Start: 0x%02X, Type: 0x%02X, Data: 0x%02X, End: 0x%02X",
                            msg.start, msg.type, msg.data, msg.end);
                }
                
                // Process message
                switch (msg.type) {
                    case RESP_STATE: {
                        std_msgs::UInt16 state_msg;
                        state_msg.data = msg.data;
                        state_pub_.publish(state_msg);
                        break;
                    }
                    case RESP_ERROR: {
                        ROS_ERROR("Received error from ESP32: 0x%02X", msg.data);
                        break;
                    }
                    case CMD_HEARTBEAT:
                        last_heartbeat_ = std::chrono::steady_clock::now();
                        if (DEBUG) {
                            ROS_DEBUG("Received heartbeat");
                        }
                        break;
                }
                
                // Remove processed message
                rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + sizeof(SimpleMessage));
            } else {
                // Invalid message, remove start byte and continue
                rx_buffer_.pop_front();
            }
        }
    }

    void checkHeartbeat() {
        while (running_) {
            auto now = std::chrono::steady_clock::now();
            if (now - last_heartbeat_ > HEARTBEAT_TIMEOUT) {
                ROS_WARN("No heartbeat received from ESP32 for more than 2 seconds");
                handleSerialError();
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

public:
    ServoSerialBridge() : nh_("~"), serial_(io_), running_(false) {
        // Get parameters
        nh_.param<std::string>("port", port_, "/dev/ttyUSB1");
        nh_.param<int>("baud_rate", baud_rate_, 115200);

        // Setup ROS subscribers and publishers
        cmd_sub_ = nh_.subscribe("/sroi_gripper/command", 1, &ServoSerialBridge::servoCb, this);
        state_pub_ = nh_.advertise<std_msgs::UInt16>("/sroi_gripper/state", 1);

        // Start threads
        running_ = true;
        last_heartbeat_ = std::chrono::steady_clock::now();
        read_thread_ = std::thread(&ServoSerialBridge::readSerial, this);
        std::thread heartbeat_thread(&ServoSerialBridge::checkHeartbeat, this);
        heartbeat_thread.detach();
    }

    ~ServoSerialBridge() {
        running_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        if (serial_.is_open()) {
            serial_.close();
        }
    }

    void servoCb(const std_msgs::UInt16::ConstPtr& msg) {
        uint8_t angle = static_cast<uint8_t>(msg->data);
        sendMessage(CMD_SET_ANGLE, angle);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "servo_serial_bridge_cpp");
    ServoSerialBridge bridge;
    ros::spin();
    return 0;
}

