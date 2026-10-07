#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <exception>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include "iwalk_hardware/vesc_protocol.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using iwalk_hardware::CanMessage;

std::atomic_bool running{true};

void signal_handler(int)
{
  running.store(false);
}

struct Options
{
  std::string can_interface = "vcan0";
  std::string serial_port = "/dev/ttyUSB0";
  uint32_t can_bitrate = 500000;
  uint32_t serial_baudrate = 2000000;
};

uint32_t parse_u32(const std::string & option, const char * text)
{
  char * end = nullptr;
  errno = 0;
  const unsigned long value = std::strtoul(text, &end, 10);
  if (errno == ERANGE || end == text || *end != '\0' || value > UINT32_MAX) {
    throw std::invalid_argument(option + " expects an unsigned integer");
  }
  return static_cast<uint32_t>(value);
}

void print_usage(const char * executable)
{
  std::cout << "Usage: " << executable << " [options]\n"
            << "  --can-interface NAME       SocketCAN side (default: vcan0)\n"
            << "  --serial-port DEVICE       adapter port (default: /dev/ttyUSB0)\n"
            << "  --can-bitrate BPS          physical CAN bitrate (default: 500000)\n"
            << "  --serial-baudrate BAUD     USB serial baudrate (default: 2000000)\n";
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--help" || option == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value for " + option);
    }
    const char * value = argv[++index];
    if (option == "--can-interface") {
      options.can_interface = value;
    } else if (option == "--serial-port") {
      options.serial_port = value;
    } else if (option == "--can-bitrate") {
      options.can_bitrate = parse_u32(option, value);
    } else if (option == "--serial-baudrate") {
      options.serial_baudrate = parse_u32(option, value);
    } else {
      throw std::invalid_argument("unknown option " + option);
    }
  }
  if (options.can_interface.empty() || options.can_interface.size() >= IFNAMSIZ) {
    throw std::invalid_argument("--can-interface is empty or too long");
  }
  if (options.serial_port.empty()) {
    throw std::invalid_argument("--serial-port is empty");
  }
  // Validate against the exact bitrate map used by python-can's Seeed backend.
  (void)iwalk_hardware::make_seeed_init_frame(options.can_bitrate);
  return options;
}

speed_t serial_speed(uint32_t baudrate)
{
  switch (baudrate) {
    case 115200U: return B115200;
#ifdef B460800
    case 460800U: return B460800;
#endif
#ifdef B921600
    case 921600U: return B921600;
#endif
#ifdef B1000000
    case 1000000U: return B1000000;
#endif
#ifdef B2000000
    case 2000000U: return B2000000;
#endif
    default:
      throw std::invalid_argument("unsupported termios serial baudrate");
  }
}

int open_serial(const Options & options)
{
  const int descriptor = ::open(
    options.serial_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (descriptor < 0) {
    throw std::runtime_error(
            "cannot open " + options.serial_port + ": " + std::strerror(errno));
  }
  if (::flock(descriptor, LOCK_EX | LOCK_NB) < 0) {
    const std::string error = std::strerror(errno);
    ::close(descriptor);
    throw std::runtime_error(
            "cannot exclusively own " + options.serial_port + ": " + error);
  }
  if (::ioctl(descriptor, TIOCEXCL) < 0) {
    const std::string error = std::strerror(errno);
    ::close(descriptor);
    throw std::runtime_error(
            "cannot set exclusive tty ownership on " + options.serial_port + ": " + error);
  }

  termios settings{};
  if (::tcgetattr(descriptor, &settings) < 0) {
    const std::string error = std::strerror(errno);
    ::close(descriptor);
    throw std::runtime_error("tcgetattr failed: " + error);
  }
  ::cfmakeraw(&settings);
  settings.c_cflag |= CLOCAL | CREAD;
  settings.c_cflag &= ~CRTSCTS;
  settings.c_cflag &= ~CSTOPB;
  settings.c_cflag &= ~PARENB;
  settings.c_cflag = (settings.c_cflag & ~CSIZE) | CS8;
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;
  const speed_t speed = serial_speed(options.serial_baudrate);
  if (::cfsetispeed(&settings, speed) < 0 || ::cfsetospeed(&settings, speed) < 0 ||
    ::tcsetattr(descriptor, TCSANOW, &settings) < 0)
  {
    const std::string error = std::strerror(errno);
    ::close(descriptor);
    throw std::runtime_error("cannot configure serial port: " + error);
  }
  return descriptor;
}

int open_can(const std::string & interface_name)
{
  const int descriptor = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (descriptor < 0) {
    throw std::runtime_error("cannot create SocketCAN socket: " + std::string(std::strerror(errno)));
  }
  const int receive_own_messages = 0;
  if (::setsockopt(
      descriptor, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
      &receive_own_messages, sizeof(receive_own_messages)) < 0)
  {
    const std::string error = std::strerror(errno);
    ::close(descriptor);
    throw std::runtime_error("cannot disable CAN_RAW_RECV_OWN_MSGS: " + error);
  }
  const unsigned int interface_index = ::if_nametoindex(interface_name.c_str());
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = static_cast<int>(interface_index);
  if (interface_index == 0 ||
    ::bind(descriptor, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
  {
    const std::string error = interface_index == 0 ? "interface not found" : std::strerror(errno);
    ::close(descriptor);
    throw std::runtime_error("cannot bind " + interface_name + ": " + error);
  }
  return descriptor;
}

void write_with_deadline(int descriptor, const uint8_t * data, std::size_t size)
{
  const auto deadline = Clock::now() + std::chrono::milliseconds(500);
  std::size_t offset = 0;
  while (offset < size) {
    const ssize_t written = ::write(descriptor, data + offset, size - offset);
    if (written > 0) {
      offset += static_cast<std::size_t>(written);
      continue;
    }
    if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      throw std::runtime_error("serial initialization write failed: " + std::string(std::strerror(errno)));
    }
    if (Clock::now() >= deadline) {
      throw std::runtime_error("serial initialization write timed out");
    }
    pollfd descriptor_to_poll{descriptor, POLLOUT, 0};
    (void)::poll(&descriptor_to_poll, 1, 10);
  }
}

CanMessage from_linux_frame(const can_frame & frame)
{
  CanMessage message;
  message.extended = (frame.can_id & CAN_EFF_FLAG) != 0;
  message.remote = (frame.can_id & CAN_RTR_FLAG) != 0;
  message.id = frame.can_id & (message.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  message.dlc = std::min<uint8_t>(frame.can_dlc, 8U);
  std::copy_n(frame.data, message.dlc, message.data.begin());
  return message;
}

can_frame to_linux_frame(const CanMessage & message)
{
  can_frame frame{};
  frame.can_id = message.id;
  if (message.extended) {
    frame.can_id |= CAN_EFF_FLAG;
  }
  if (message.remote) {
    frame.can_id |= CAN_RTR_FLAG;
  }
  frame.can_dlc = message.dlc;
  std::copy_n(message.data.begin(), message.dlc, frame.data);
  return frame;
}

class Bridge
{
public:
  explicit Bridge(const Options & options)
  : serial_(open_serial(options)), can_(open_can(options.can_interface))
  {
    const auto initialization = iwalk_hardware::make_seeed_init_frame(options.can_bitrate);
    write_with_deadline(serial_, initialization.data(), initialization.size());
    std::cout << "Waveshare bridge active: " << options.can_interface << " <-> "
              << options.serial_port << " (serial " << options.serial_baudrate
              << ", CAN " << options.can_bitrate << ", EXT, normal mode)\n"
              << "A successful vcan/serial write is not proof of physical CAN acknowledgement.\n";
  }

  ~Bridge()
  {
    if (can_ >= 0) {
      ::close(can_);
    }
    if (serial_ >= 0) {
      ::close(serial_);
    }
  }

  void run()
  {
    auto next_heartbeat = Clock::now();
    while (running.load()) {
      const auto now = Clock::now();
      if (serial_tx_ && now - serial_tx_->created > std::chrono::milliseconds(100)) {
        throw std::runtime_error(
                "serial output stalled for more than 100 ms; exiting without replay/reconnect");
      }
      if (can_tx_.size() > kMaximumCanQueue) {
        throw std::runtime_error("SocketCAN output queue exceeded its bound");
      }

      pollfd descriptors[2]{};
      descriptors[0].fd = serial_;
      descriptors[0].events = POLLIN | (serial_tx_ ? POLLOUT : 0);
      descriptors[1].fd = can_;
      descriptors[1].events = (serial_tx_ ? 0 : POLLIN) | (!can_tx_.empty() ? POLLOUT : 0);
      const int result = ::poll(descriptors, 2, 10);
      if (result < 0 && errno != EINTR) {
        throw std::runtime_error("poll failed: " + std::string(std::strerror(errno)));
      }
      if ((descriptors[0].revents | descriptors[1].revents) & (POLLERR | POLLHUP | POLLNVAL)) {
        throw std::runtime_error("serial or SocketCAN endpoint disconnected");
      }

      if ((descriptors[0].revents & POLLIN) != 0) {
        read_serial();
      }
      if ((descriptors[1].revents & POLLIN) != 0) {
        read_can();
      }
      if (serial_tx_ && (descriptors[0].revents & POLLOUT) != 0) {
        write_serial();
      }
      if (!can_tx_.empty() && (descriptors[1].revents & POLLOUT) != 0) {
        write_can_queue();
      }

      if (Clock::now() >= next_heartbeat) {
        queue_can(to_linux_frame(iwalk_hardware::make_bridge_heartbeat(heartbeat_sequence_++)));
        next_heartbeat += std::chrono::milliseconds(50);
        if (next_heartbeat < Clock::now()) {
          next_heartbeat = Clock::now() + std::chrono::milliseconds(50);
        }
        write_can_queue();
      }
    }
  }

private:
  struct SerialPending
  {
    std::vector<uint8_t> bytes;
    std::size_t offset = 0;
    Clock::time_point created = Clock::now();
  };

  static constexpr std::size_t kMaximumCanQueue = 256;

  void read_can()
  {
    for (int count = 0; count < 128 && !serial_tx_; ++count) {
      can_frame frame{};
      const ssize_t size = ::read(can_, &frame, sizeof(frame));
      if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        return;
      }
      if (size < 0 && errno == EINTR) {
        continue;
      }
      if (size != static_cast<ssize_t>(sizeof(frame))) {
        throw std::runtime_error("SocketCAN read failed or returned a truncated frame");
      }
      if ((frame.can_id & CAN_ERR_FLAG) != 0) {
        throw std::runtime_error("SocketCAN error frame received");
      }
      const CanMessage message = from_linux_frame(frame);
      if (message.id == iwalk_hardware::kBridgeHeartbeatCanId) {
        continue;
      }
      // The adapter is initialized for extended frames; do not silently reinterpret SFF traffic.
      if (!message.extended || message.dlc > 8) {
        continue;
      }
      serial_tx_ = SerialPending{iwalk_hardware::serialize_seeed_frame(message)};
      write_serial();
    }
  }

  void write_serial()
  {
    if (!serial_tx_) {
      return;
    }
    auto & pending = *serial_tx_;
    const ssize_t written = ::write(
      serial_, pending.bytes.data() + pending.offset, pending.bytes.size() - pending.offset);
    if (written > 0) {
      pending.offset += static_cast<std::size_t>(written);
      if (pending.offset == pending.bytes.size()) {
        serial_tx_.reset();
      }
      return;
    }
    if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      return;
    }
    throw std::runtime_error("serial write failed: " + std::string(std::strerror(errno)));
  }

  void read_serial()
  {
    std::array<uint8_t, 1024> bytes{};
    for (int count = 0; count < 16; ++count) {
      const ssize_t size = ::read(serial_, bytes.data(), bytes.size());
      if (size == 0 || (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
        return;
      }
      if (size < 0 && errno == EINTR) {
        continue;
      }
      if (size < 0) {
        throw std::runtime_error("serial read failed: " + std::string(std::strerror(errno)));
      }
      for (const auto & message : parser_.feed(bytes.data(), static_cast<std::size_t>(size))) {
        // EXT is selected in the adapter initialization. Ignore unexpected standard frames.
        if (message.extended) {
          queue_can(to_linux_frame(message));
        }
      }
      if (static_cast<std::size_t>(size) < bytes.size()) {
        return;
      }
    }
  }

  void queue_can(const can_frame & frame)
  {
    if (can_tx_.size() >= kMaximumCanQueue) {
      throw std::runtime_error("SocketCAN output queue is full");
    }
    can_tx_.push_back(frame);
  }

  void write_can_queue()
  {
    while (!can_tx_.empty()) {
      const ssize_t written = ::write(can_, &can_tx_.front(), sizeof(can_frame));
      if (written == static_cast<ssize_t>(sizeof(can_frame))) {
        can_tx_.pop_front();
        continue;
      }
      if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        return;
      }
      throw std::runtime_error("SocketCAN write failed: " + std::string(std::strerror(errno)));
    }
  }

  int serial_ = -1;
  int can_ = -1;
  iwalk_hardware::SeeedSerialParser parser_;
  std::optional<SerialPending> serial_tx_;
  std::deque<can_frame> can_tx_;
  uint16_t heartbeat_sequence_ = 0;
};

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parse_options(argc, argv);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    Bridge bridge(options);
    bridge.run();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "vesc_waveshare_bridge: " << error.what() << '\n';
    return 1;
  }
}
