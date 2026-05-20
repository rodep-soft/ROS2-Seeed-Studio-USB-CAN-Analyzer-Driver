#include "seeed_usb_can_analyzer_driver/serial_protocol.hpp"

#include <algorithm>
#include <cerrno>
#include <stdexcept>
#include <system_error>
#include <utility>

#include <sys/ioctl.h>
#include <termios.h>

#if defined(__APPLE__)
#include <IOKit/serial/ioss.h>
#endif

namespace seeed_usb_can
{

namespace
{

speed_t to_posix_baud(int baud)
{
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
#if defined(B500000)
    case 500000:
      return B500000;
#endif
#if defined(B576000)
    case 576000:
      return B576000;
#endif
#if defined(B921600)
    case 921600:
      return B921600;
#endif
#if defined(B1000000)
    case 1000000:
      return B1000000;
#endif
#if defined(B1152000)
    case 1152000:
      return B1152000;
#endif
#if defined(B1500000)
    case 1500000:
      return B1500000;
#endif
#if defined(B2000000)
    case 2000000:
      return B2000000;
#endif
    default:
      throw std::invalid_argument("Unsupported serial_baud: " + std::to_string(baud));
  }
}

void configure_serial_port(int fd, int baud)
{
  struct termios tty {};
  if (tcgetattr(fd, &tty) != 0) {
    throw std::system_error(errno, std::generic_category(), "tcgetattr failed");
  }

  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
#if defined(CRTSCTS)
  tty.c_cflag &= ~CRTSCTS;
#endif
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 0;

#if defined(__APPLE__)
  if (cfsetispeed(&tty, B9600) != 0 || cfsetospeed(&tty, B9600) != 0) {
    throw std::system_error(errno, std::generic_category(), "cfset*speed failed");
  }
#else
  const auto speed = to_posix_baud(baud);
  if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
    throw std::system_error(errno, std::generic_category(), "cfset*speed failed");
  }
#endif

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    throw std::system_error(errno, std::generic_category(), "tcsetattr failed");
  }

#if defined(__APPLE__)
  speed_t ios_speed = static_cast<speed_t>(baud);
  if (ioctl(fd, IOSSIOSPEED, &ios_speed) == -1) {
    throw std::system_error(errno, std::generic_category(), "ioctl(IOSSIOSPEED) failed");
  }
#endif

  if (tcflush(fd, TCIOFLUSH) != 0) {
    throw std::system_error(errno, std::generic_category(), "tcflush failed");
  }
}

}  // namespace

UsbCanSerialDriver::UsbCanSerialDriver()
: serial_port_(io_context_)
{
}

UsbCanSerialDriver::~UsbCanSerialDriver()
{
  close();
}

void UsbCanSerialDriver::open(const SerialDriverConfig & config)
{
  try {
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (serial_port_.is_open()) {
        throw std::runtime_error("Serial port already open");
      }

      serial_port_.open(config.usb_path);
      configure_serial_port(serial_port_.native_handle(), config.serial_baud);
    }

    auto init = build_initialization_frame(
      config.can_baud_code,
      config.tx_extended,
      config.filter_id,
      config.mask_id,
      config.operation_mode);

    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      boost::asio::write(serial_port_, boost::asio::buffer(init));
    }

    running_.store(true);
    start_async_read();
    io_thread_ = std::thread([this]() { io_context_.run(); });
  } catch (...) {
    boost::system::error_code ec;
    serial_port_.cancel(ec);
    serial_port_.close(ec);
    running_.store(false);
    throw;
  }
}

void UsbCanSerialDriver::close()
{
  running_.store(false);
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    boost::system::error_code ec;
    serial_port_.cancel(ec);
    serial_port_.close(ec);
  }
  io_context_.stop();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  io_context_.restart();
}

bool UsbCanSerialDriver::is_open() const
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  return serial_port_.is_open();
}

void UsbCanSerialDriver::set_receive_callback(std::function<void(const CanFrame &)> callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  on_receive_ = std::move(callback);
}

void UsbCanSerialDriver::send_frame(const CanFrame & frame)
{
  auto bytes = encode_can_frame(frame);
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (!serial_port_.is_open()) {
    throw std::runtime_error("Serial port is not open");
  }
  boost::asio::write(serial_port_, boost::asio::buffer(bytes));
}

std::vector<uint8_t> UsbCanSerialDriver::build_initialization_frame(
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

std::vector<uint8_t> UsbCanSerialDriver::encode_can_frame(const CanFrame & frame)
{
  if (frame.dlc > 8U || frame.data.size() > 8U) {
    throw std::invalid_argument("CAN data payload must be 0..8 bytes");
  }
  if (frame.dlc != frame.data.size()) {
    throw std::invalid_argument("CAN frame dlc must match data length");
  }

  const uint8_t dlc = frame.dlc;
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
    out.push_back(static_cast<uint8_t>((frame.id >> 0U) & 0xFFU));
    out.push_back(static_cast<uint8_t>((frame.id >> 8U) & 0xFFU));
    out.push_back(static_cast<uint8_t>((frame.id >> 16U) & 0xFFU));
    out.push_back(static_cast<uint8_t>((frame.id >> 24U) & 0xFFU));
  } else {
    out.push_back(static_cast<uint8_t>((frame.id >> 0U) & 0xFFU));
    out.push_back(static_cast<uint8_t>((frame.id >> 8U) & 0xFFU));
  }

  out.insert(out.end(), frame.data.begin(), frame.data.begin() + dlc);
  out.push_back(0x00U);
  out.push_back(0x55U);

  return out;
}

std::optional<CanFrame> UsbCanSerialDriver::try_extract_frame(std::vector<uint8_t> & buffer)
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
    frame.dlc = dlc;

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
    frame.data.assign(
      buffer.begin() + static_cast<std::ptrdiff_t>(data_offset),
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
    // Move forward by one byte to re-synchronize stream parsing on the next frame boundary.
    buffer.erase(buffer.begin());
  }
  return std::nullopt;
}

uint8_t UsbCanSerialDriver::checksum_8bit(
  const std::vector<uint8_t> & frame, size_t begin_inclusive, size_t end_inclusive)
{
  uint32_t sum = 0;
  for (size_t i = begin_inclusive; i <= end_inclusive && i < frame.size(); ++i) {
    sum += frame[i];
  }
  return static_cast<uint8_t>(sum & 0xFFU);
}

void UsbCanSerialDriver::store_u32_le(std::vector<uint8_t> & dst, size_t offset, uint32_t value)
{
  dst[offset + 0U] = static_cast<uint8_t>((value >> 0U) & 0xFFU);
  dst[offset + 1U] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
  dst[offset + 2U] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
  dst[offset + 3U] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}

void UsbCanSerialDriver::start_async_read()
{
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!serial_port_.is_open()) {
      return;
    }
  }

  serial_port_.async_read_some(
    boost::asio::buffer(read_buffer_),
    [this](const boost::system::error_code & ec, std::size_t bytes_transferred) {
      if (ec) {
        if (ec == boost::asio::error::operation_aborted) {
          return;
        }
        if (running_.load() && is_open()) {
          start_async_read();
        }
        return;
      }

      std::vector<CanFrame> extracted;
      {
        std::lock_guard<std::mutex> serial_lock(serial_mutex_);
        rx_bytes_.insert(
          rx_bytes_.end(),
          read_buffer_.begin(),
          read_buffer_.begin() + static_cast<std::ptrdiff_t>(bytes_transferred));

        while (true) {
          auto frame = try_extract_frame(rx_bytes_);
          if (!frame) {
            break;
          }
          extracted.push_back(*frame);
        }
      }

      std::function<void(const CanFrame &)> callback;
      {
        std::lock_guard<std::mutex> callback_lock(callback_mutex_);
        callback = on_receive_;
      }

      if (callback) {
        for (const auto & frame : extracted) {
          callback(frame);
        }
      }

      if (running_.load() && is_open()) {
        start_async_read();
      }
    });
}

}  // namespace seeed_usb_can
