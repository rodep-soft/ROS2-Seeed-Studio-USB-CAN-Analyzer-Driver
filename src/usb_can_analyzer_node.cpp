#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "seeed_usb_can_analyzer_driver/msg/can_frame.hpp"
#include "seeed_usb_can_analyzer_driver/serial_protocol.hpp"

namespace seeed_usb_can
{

class UsbCanAnalyzerNode : public rclcpp::Node
{
public:
  UsbCanAnalyzerNode()
  : Node("usb_can_analyzer_node")
  {
    const std::string usb_path = declare_parameter<std::string>("usb_path", "/dev/ttyUSB0");
    const int serial_baud = declare_parameter<int>("serial_baud", 2000000);

    const int bitrate = declare_parameter<int>("bitrate", 500000);
    const bool tx_extended = declare_parameter<bool>("tx_extended", false);
    const int64_t filter_id = declare_parameter<int64_t>("filter_id", 0);
    const int64_t mask_id = declare_parameter<int64_t>("mask_id", 0);
    const int operation_mode = declare_parameter<int>("operation_mode", 0);

    if (operation_mode < 0 || operation_mode > 3) {
      throw std::invalid_argument(
        "Parameter 'operation_mode' must be 0..3 "
        "(0=normal, 1=loopback, 2=silent, 3=loopback+silent)");
    }

    const auto can_baud_code = to_can_baud_code(bitrate);

    publisher_ = create_publisher<seeed_usb_can_analyzer_driver::msg::CanFrame>(
      "/ssuca/receive", 100);
    subscription_ = create_subscription<seeed_usb_can_analyzer_driver::msg::CanFrame>(
      "/ssuca/transmit",
      100,
      [this](const seeed_usb_can_analyzer_driver::msg::CanFrame::SharedPtr msg) {
        this->handle_transmit(*msg);
      });

    SerialDriverConfig config;
    config.usb_path = usb_path;
    config.serial_baud = serial_baud;
    config.can_baud_code = can_baud_code;
    config.tx_extended = tx_extended;
    config.filter_id = static_cast<uint32_t>(filter_id);
    config.mask_id = static_cast<uint32_t>(mask_id);
    config.operation_mode = static_cast<uint8_t>(operation_mode);

    serial_driver_.set_receive_callback([this](const CanFrame & frame) {
      auto msg = seeed_usb_can_analyzer_driver::msg::CanFrame();
      msg.id = frame.id;
      msg.extended = frame.extended;
      msg.remote = frame.remote;
      msg.dlc = frame.dlc;
      msg.data = frame.data;
      publisher_->publish(msg);
    });

    try {
      serial_driver_.open(config);
      RCLCPP_INFO(
        get_logger(),
        "USB-CAN analyzer initialized: usb_path=%s serial_baud=%d bitrate=%d",
        usb_path.c_str(),
        serial_baud,
        bitrate);
    } catch (const std::exception & e) {
      throw std::runtime_error(
        "Failed to open/initialize serial device '" + usb_path + "': " + e.what());
    }
  }

  ~UsbCanAnalyzerNode() override
  {
    serial_driver_.close();
  }

private:
  static uint8_t to_can_baud_code(int bitrate)
  {
    switch (bitrate) {
      case 1000000:
        return 0x01;
      case 800000:
        return 0x02;
      case 500000:
        return 0x03;
      case 400000:
        return 0x04;
      case 250000:
        return 0x05;
      case 200000:
        return 0x06;
      case 125000:
        return 0x07;
      case 100000:
        return 0x08;
      case 50000:
        return 0x09;
      case 20000:
        return 0x0A;
      case 10000:
        return 0x0B;
      case 5000:
        return 0x0C;
      default:
        throw std::invalid_argument(
          "Parameter 'bitrate' must be one of: "
          "1000000, 800000, 500000, 400000, 250000, 200000, 125000, "
          "100000, 50000, 20000, 10000, 5000");
    }
  }

  void handle_transmit(const seeed_usb_can_analyzer_driver::msg::CanFrame & msg)
  {
    if (msg.dlc > 8U || msg.data.size() > 8U) {
      RCLCPP_WARN(get_logger(), "Ignoring /ssuca/transmit frame with DLC/data > 8");
      return;
    }

    CanFrame frame;
    frame.id = msg.id;
    frame.extended = msg.extended;
    frame.remote = msg.remote;

    const auto payload_len = std::min<std::size_t>(msg.data.size(), msg.dlc);
    frame.dlc = static_cast<uint8_t>(payload_len);
    frame.data.assign(msg.data.begin(), msg.data.begin() + static_cast<std::ptrdiff_t>(payload_len));

    try {
      serial_driver_.send_frame(frame);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to transmit CAN frame: %s", e.what());
    }
  }

  UsbCanSerialDriver serial_driver_;
  rclcpp::Publisher<seeed_usb_can_analyzer_driver::msg::CanFrame>::SharedPtr publisher_;
  rclcpp::Subscription<seeed_usb_can_analyzer_driver::msg::CanFrame>::SharedPtr subscription_;
};

}  // namespace seeed_usb_can

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<seeed_usb_can::UsbCanAnalyzerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    auto logger = rclcpp::get_logger("usb_can_analyzer_node");
    RCLCPP_FATAL(logger, "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
