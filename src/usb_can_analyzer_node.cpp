#include <algorithm>
#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>

namespace seeed_usb_can
{
struct CanFrame
{
  uint32_t id{0};
  bool extended{false};
  bool remote{false};
  std::vector<uint8_t> data;
};

class UsbCanProtocol
{
public:
  static std::vector<uint8_t> build_initialization_frame(
    uint8_t can_baud_code,
    bool tx_extended,
    uint32_t filter_id,
    uint32_t mask_id,
    uint8_t operation_mode)
  {
    std::vector<uint8_t> frame(20U, 0x00U);
    frame[0] = 0xAAU;
    frame[1] = 0x55U;
    frame[2] = 0x12U;
    frame[3] = can_baud_code;
    frame[4] = tx_extended ? 0x02U : 0x01U;

    store_u32_le(frame, 5U, filter_id);
    store_u32_le(frame, 9U, mask_id);

    frame[13] = operation_mode;
    frame[14] = 0x01U;

    frame[19] = checksum_8bit(frame, 2U, 18U);
    return frame;
  }

  static std::vector<uint8_t> encode_can_frame(const CanFrame & frame)
  {
    const uint8_t dlc = static_cast<uint8_t>(std::min<size_t>(frame.data.size(), 8U));
    const uint8_t id_len = frame.extended ? 4U : 2U;

    std::vector<uint8_t> out;
    out.reserve(static_cast<size_t>(1U + 1U + id_len + dlc + 1U + 1U));
    out.push_back(0xAAU);

    uint8_t info = 0xC0U;
    if (frame.extended) {
      info |= 0x20U;
    }
    if (frame.remote) {
      info |= 0x10U;
    }
    info |= dlc;
    out.push_back(info);

    if (frame.extended) {
      out.push_back(static_cast<uint8_t>((frame.id >> 0) & 0xFFU));
      out.push_back(static_cast<uint8_t>((frame.id >> 8) & 0xFFU));
      out.push_back(static_cast<uint8_t>((frame.id >> 16) & 0xFFU));
      out.push_back(static_cast<uint8_t>((frame.id >> 24) & 0xFFU));
    } else {
      out.push_back(static_cast<uint8_t>((frame.id >> 0) & 0xFFU));
      out.push_back(static_cast<uint8_t>((frame.id >> 8) & 0xFFU));
    }

    out.insert(out.end(), frame.data.begin(), frame.data.begin() + dlc);

    out.push_back(0x00U);
    out.push_back(0x55U);
    return out;
  }

  static std::optional<CanFrame> try_extract_frame(std::vector<uint8_t> & buffer)
  {
    const auto start_it = std::find(buffer.begin(), buffer.end(), 0xAAU);
    if (start_it == buffer.end()) {
      buffer.clear();
      return std::nullopt;
    }
    if (start_it != buffer.begin()) {
      buffer.erase(buffer.begin(), start_it);
    }

    if (buffer.size() < 5U) {
      return std::nullopt;
    }

    const uint8_t info = buffer[1];
    if ((info & 0xC0U) != 0xC0U) {
      buffer.erase(buffer.begin());
      return std::nullopt;
    }

    const bool extended = (info & 0x20U) != 0U;
    const bool remote = (info & 0x10U) != 0U;
    const uint8_t dlc = static_cast<uint8_t>(info & 0x0FU);
    if (dlc > 8U) {
      buffer.erase(buffer.begin());
      return std::nullopt;
    }

    const size_t id_len = extended ? 4U : 2U;
    const size_t without_padding = 1U + 1U + id_len + dlc + 1U;
    const size_t with_padding = without_padding + 1U;

    auto parse_with_length = [&](size_t total_len) -> std::optional<CanFrame> {
      if (buffer.size() < total_len) {
        return std::nullopt;
      }
      if (buffer[total_len - 1U] != 0x55U) {
        return std::nullopt;
      }

      CanFrame frame;
      frame.extended = extended;
      frame.remote = remote;

      if (extended) {
        frame.id =
          static_cast<uint32_t>(buffer[2]) |
          (static_cast<uint32_t>(buffer[3]) << 8U) |
          (static_cast<uint32_t>(buffer[4]) << 16U) |
          (static_cast<uint32_t>(buffer[5]) << 24U);
      } else {
        frame.id = static_cast<uint32_t>(buffer[2]) |
          (static_cast<uint32_t>(buffer[3]) << 8U);
      }

      const size_t data_offset = 2U + id_len;
      frame.data.assign(buffer.begin() + static_cast<std::ptrdiff_t>(data_offset),
        buffer.begin() + static_cast<std::ptrdiff_t>(data_offset + dlc));

      buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(total_len));
      return frame;
    };

    if (auto frame = parse_with_length(with_padding)) {
      return frame;
    }
    if (auto frame = parse_with_length(without_padding)) {
      return frame;
    }

    if (buffer.size() > with_padding) {
      buffer.erase(buffer.begin());
    }
    return std::nullopt;
  }

private:
  static uint8_t checksum_8bit(
    const std::vector<uint8_t> & frame, size_t begin_inclusive, size_t end_inclusive)
  {
    uint32_t sum = 0;
    for (size_t i = begin_inclusive; i <= end_inclusive && i < frame.size(); ++i) {
      sum += frame[i];
    }
    return static_cast<uint8_t>(sum & 0xFFU);
  }

  static void store_u32_le(std::vector<uint8_t> & dst, size_t offset, uint32_t value)
  {
    dst[offset + 0U] = static_cast<uint8_t>((value >> 0U) & 0xFFU);
    dst[offset + 1U] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    dst[offset + 2U] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
    dst[offset + 3U] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
  }
};

class UsbCanAnalyzerNode : public rclcpp::Node
{
public:
  UsbCanAnalyzerNode()
  : Node("usb_can_analyzer_node"),
    io_context_(),
    serial_port_(io_context_)
  {
    const std::string serial_device = declare_parameter<std::string>("serial_device", "/dev/ttyUSB0");
    const int serial_baud = declare_parameter<int>("serial_baud", 2000000);

    const int can_baud_code = declare_parameter<int>("can_baud_code", 3);
    const bool tx_extended = declare_parameter<bool>("tx_extended", false);
    const int64_t filter_id = declare_parameter<int64_t>("filter_id", 0);
    const int64_t mask_id = declare_parameter<int64_t>("mask_id", 0);
    const int operation_mode = declare_parameter<int>("operation_mode", 0);

    try {
      serial_port_.open(serial_device);
      serial_port_.set_option(boost::asio::serial_port_base::baud_rate(serial_baud));
      serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
      serial_port_.set_option(
        boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      serial_port_.set_option(
        boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
      serial_port_.set_option(
        boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

      auto init = UsbCanProtocol::build_initialization_frame(
        static_cast<uint8_t>(can_baud_code),
        tx_extended,
        static_cast<uint32_t>(filter_id),
        static_cast<uint32_t>(mask_id),
        static_cast<uint8_t>(operation_mode));

      boost::asio::write(serial_port_, boost::asio::buffer(init));
      RCLCPP_INFO(get_logger(), "USB-CAN analyzer initialized on %s", serial_device.c_str());

      start_async_read();
      io_thread_ = std::thread([this]() { io_context_.run(); });
    } catch (const std::exception & e) {
      throw std::runtime_error(std::string("Failed to open/initialize serial device: ") + e.what());
    }
  }

  ~UsbCanAnalyzerNode() override
  {
    boost::system::error_code ec;
    serial_port_.cancel(ec);
    serial_port_.close(ec);
    io_context_.stop();
    if (io_thread_.joinable()) {
      io_thread_.join();
    }
  }

private:
  void start_async_read()
  {
    serial_port_.async_read_some(
      boost::asio::buffer(read_buffer_),
      [this](const boost::system::error_code & ec, std::size_t bytes_transferred) {
        if (ec) {
          if (ec == boost::asio::error::operation_aborted) {
            return;
          }
          RCLCPP_WARN(get_logger(), "Serial read error: %s", ec.message().c_str());
          if (serial_port_.is_open()) {
            start_async_read();
          }
          return;
        }

        {
          std::lock_guard<std::mutex> lock(rx_mutex_);
          rx_bytes_.insert(
            rx_bytes_.end(),
            read_buffer_.begin(),
            read_buffer_.begin() + static_cast<std::ptrdiff_t>(bytes_transferred));

          while (true) {
            auto frame = UsbCanProtocol::try_extract_frame(rx_bytes_);
            if (!frame) {
              break;
            }
            RCLCPP_DEBUG(
              get_logger(),
              "RX CAN frame: id=0x%08X ext=%d rtr=%d dlc=%zu",
              frame->id,
              frame->extended,
              frame->remote,
              frame->data.size());
          }
        }

        start_async_read();
      });
  }

  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_port_;
  std::thread io_thread_;

  std::array<uint8_t, 256> read_buffer_{};
  std::vector<uint8_t> rx_bytes_;
  std::mutex rx_mutex_;
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
