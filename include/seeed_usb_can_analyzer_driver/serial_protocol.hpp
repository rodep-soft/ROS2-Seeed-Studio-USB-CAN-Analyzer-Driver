#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

namespace seeed_usb_can
{

struct CanFrame
{
  uint32_t id{0};
  bool extended{false};
  bool remote{false};
  uint8_t dlc{0};
  std::vector<uint8_t> data;
};

struct SerialDriverConfig
{
  std::string usb_path{"/dev/ttyUSB0"};
  int serial_baud{2000000};
  uint8_t can_baud_code{0x03};
  bool tx_extended{false};
  uint32_t filter_id{0};
  uint32_t mask_id{0};
  uint8_t operation_mode{0x00};
};

class UsbCanSerialDriver
{
public:
  static constexpr std::size_t READ_BUFFER_SIZE = 256U;

  UsbCanSerialDriver();
  ~UsbCanSerialDriver();

  void open(const SerialDriverConfig & config);
  void close();
  bool is_open() const;

  void set_receive_callback(std::function<void(const CanFrame &)> callback);
  void send_frame(const CanFrame & frame);

private:
  static std::vector<uint8_t> build_initialization_frame(
    uint8_t can_baud_code,
    bool tx_extended,
    uint32_t filter_id,
    uint32_t mask_id,
    uint8_t operation_mode);

  static std::vector<uint8_t> encode_can_frame(const CanFrame & frame);
  static std::optional<CanFrame> try_extract_frame(std::vector<uint8_t> & buffer);

  static uint8_t checksum_8bit(
    const std::vector<uint8_t> & frame, size_t begin_inclusive, size_t end_inclusive);
  static void store_u32_le(std::vector<uint8_t> & dst, size_t offset, uint32_t value);

  void start_async_read();

  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_port_;
  std::thread io_thread_;

  mutable std::mutex serial_mutex_;
  std::array<uint8_t, READ_BUFFER_SIZE> read_buffer_{};
  std::vector<uint8_t> rx_bytes_;

  std::mutex callback_mutex_;
  std::function<void(const CanFrame &)> on_receive_;

  std::atomic<bool> running_{false};
};

}  // namespace seeed_usb_can
