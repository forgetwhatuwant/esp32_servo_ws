// servo_serial_bridge_cpp.cpp
#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <boost/asio.hpp>
#include <sstream>
#include <iomanip>

boost::asio::serial_port* serial;

void servoCb(const std_msgs::UInt16::ConstPtr& msg) {
  std::ostringstream ss;
  ss << "S:" << std::setw(3) << std::setfill('0') << msg->data << "\n";
  boost::asio::write(*serial, boost::asio::buffer(ss.str()));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "servo_serial_bridge_cpp");
  ros::NodeHandle nh("~");

  std::string port;
  nh.param<std::string>("port", port, "/dev/ttyUSB1");

  boost::asio::io_service io;
  boost::asio::serial_port sp(io, port);
  sp.set_option(boost::asio::serial_port_base::baud_rate(115200));
  serial = &sp;

  ros::Subscriber sub = nh.subscribe("/servo", 10, servoCb);
  ros::spin();
}

