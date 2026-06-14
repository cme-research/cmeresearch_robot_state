#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <sys/statvfs.h>
#include <sys/utsname.h>

#include <rclcpp/rclcpp.hpp>
#include <cmeresearch_msgs/msg/system_stats.hpp>

using namespace std::chrono_literals;

// ── /proc readers ───────────────────────────────────────────────────────────

static bool read_sysfs_long(const std::string & path, long & out)
{
  std::ifstream f(path);
  if (!f.is_open()) {return false;}
  f >> out;
  return f.good() || f.eof();
}

// /proc/stat first line → (idle_jiffies, total_jiffies). idle includes iowait
// so transient I/O waits are not counted as CPU busy.
static bool read_cpu_jiffies(long & idle, long & total)
{
  std::ifstream f("/proc/stat");
  if (!f.is_open()) {return false;}
  std::string label;
  std::vector<long> vals(10, 0);
  f >> label;
  for (auto & v : vals) {
    f >> v;
  }
  idle = vals[3] + vals[4];
  total = 0;
  for (auto v : vals) {
    total += v;
  }
  return true;
}

static bool read_meminfo(long & total_kb, long & avail_kb)
{
  std::ifstream f("/proc/meminfo");
  if (!f.is_open()) {return false;}
  total_kb = avail_kb = 0;
  std::string key, unit;
  long val;
  while (f >> key >> val) {
    f >> unit;
    if (key == "MemTotal:") {total_kb = val;}
    if (key == "MemAvailable:") {avail_kb = val;}
    if (total_kb && avail_kb) {break;}
  }
  return total_kb > 0;
}

// ── vcgencmd shell-out (Raspberry Pi) ───────────────────────────────────────

// Run `vcgencmd <arg>` and return stdout on success. Empty optional if the
// binary is missing or the call fails — keeps the node usable on non-Pi
// hosts (sim Docker, dev workstations) without spewing errors every cycle.
static std::optional<std::string> exec_vcgencmd(const char * arg)
{
  std::string cmd = "vcgencmd ";
  cmd += arg;
  cmd += " 2>/dev/null";
  FILE * pipe = popen(cmd.c_str(), "r");
  if (pipe == nullptr) {return std::nullopt;}
  std::array<char, 128> buf{};
  std::string out;
  while (fgets(buf.data(), buf.size(), pipe) != nullptr) {
    out.append(buf.data());
  }
  int status = pclose(pipe);
  if (status != 0 || out.empty()) {return std::nullopt;}
  return out;
}

// `vcgencmd measure_temp` returns "temp=52.1'C\n". Parse the float.
static std::optional<double> read_temp_vcgencmd()
{
  auto raw = exec_vcgencmd("measure_temp");
  if (!raw) {return std::nullopt;}
  const auto eq = raw->find('=');
  const auto apos = raw->find('\'');
  if (eq == std::string::npos || apos == std::string::npos || apos <= eq + 1) {
    return std::nullopt;
  }
  try {
    return std::stod(raw->substr(eq + 1, apos - eq - 1));
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

// `vcgencmd get_throttled` returns "throttled=0x50000\n". Parse the bitmask.
static std::optional<uint32_t> read_throttled_bits()
{
  auto raw = exec_vcgencmd("get_throttled");
  if (!raw) {return std::nullopt;}
  const auto x = raw->find("0x");
  if (x == std::string::npos) {return std::nullopt;}
  try {
    return static_cast<uint32_t>(std::stoul(raw->substr(x), nullptr, 16));
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

// sysfs thermal zone fallback for non-Pi hosts (Jetson, generic x86, sim).
static std::optional<double> read_temp_sysfs()
{
  long millideg = 0;
  if (!read_sysfs_long("/sys/class/thermal/thermal_zone0/temp", millideg)) {
    return std::nullopt;
  }
  return millideg / 1000.0;
}

// ── Host identification (read once at startup) ──────────────────────────────

// Parse /etc/os-release for `PRETTY_NAME="..."`, return value without quotes.
// Empty string if the file or the field is unavailable.
static std::string read_os_pretty_name()
{
  std::ifstream f("/etc/os-release");
  if (!f.is_open()) {return "";}
  std::string line;
  const std::string key = "PRETTY_NAME=";
  while (std::getline(f, line)) {
    if (line.rfind(key, 0) == 0) {
      std::string val = line.substr(key.size());
      if (val.size() >= 2 && val.front() == '"' && val.back() == '"') {
        val = val.substr(1, val.size() - 2);
      }
      return val;
    }
  }
  return "";
}

// `uname(2)` release field, e.g. "6.6.51+rpt-rpi-v8".
static std::string read_kernel_release()
{
  struct utsname uts{};
  if (uname(&uts) != 0) {return "";}
  return uts.release;
}

// /proc/device-tree/model on Raspberry Pi → "Raspberry Pi 5 Model B Rev 1.0".
// The file is exported from the device-tree blob and typically null-terminated;
// strip the trailing NUL plus any whitespace. Returns "" on non-Pi hosts.
static std::string read_hardware_model()
{
  std::ifstream f("/proc/device-tree/model");
  if (!f.is_open()) {return "";}
  std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  while (!s.empty() && (s.back() == '\0' || std::isspace(static_cast<unsigned char>(s.back())))) {
    s.pop_back();
  }
  return s;
}

// ── Disk usage ──────────────────────────────────────────────────────────────

// Populate the disk_* fields against the given mount point. Uses f_bavail
// (blocks available to a non-privileged user) so the numbers match `df -h`
// output rather than f_bfree which includes reserved-for-root blocks.
static bool read_disk_usage(
  const std::string & path,
  uint64_t & total_bytes,
  uint64_t & used_bytes)
{
  struct statvfs vfs{};
  if (statvfs(path.c_str(), &vfs) != 0) {return false;}
  const uint64_t frsize = vfs.f_frsize;
  total_bytes = static_cast<uint64_t>(vfs.f_blocks) * frsize;
  const uint64_t free_bytes = static_cast<uint64_t>(vfs.f_bavail) * frsize;
  used_bytes = (total_bytes > free_bytes) ? (total_bytes - free_bytes) : 0;
  return total_bytes > 0;
}

// ── Node ────────────────────────────────────────────────────────────────────

class SystemStatsNode : public rclcpp::Node
{
public:
  SystemStatsNode()
  : Node("system_stats_node")
  {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 0.2);
    // "auto" tries vcgencmd first then sysfs; "vcgencmd" / "sysfs" force one.
    temperature_source_ = declare_parameter<std::string>("temperature_source", "auto");
    disk_mount_point_ = declare_parameter<std::string>("disk_mount_point", "/");

    // Identification fields don't change at runtime — read once, copy into
    // every periodic sample so MQTT retain keeps them visible to late
    // subscribers.
    os_pretty_name_ = read_os_pretty_name();
    kernel_release_ = read_kernel_release();
    hardware_model_ = read_hardware_model();

    pub_ = create_publisher<cmeresearch_msgs::msg::SystemStats>(
      "/system_stats", rclcpp::QoS(10).transient_local());

    // Seed the CPU counters so the first publish reflects a real interval
    // instead of jiffies-since-boot.
    read_cpu_jiffies(prev_idle_, prev_total_);

    const auto period_ms = std::chrono::milliseconds(
      static_cast<int64_t>(1000.0 / std::max(publish_rate_hz_, 0.01)));
    timer_ = create_wall_timer(period_ms, [this]() {publish_stats();});

    RCLCPP_INFO(
      get_logger(),
      "system_stats_node up — publishing /system_stats at %.2f Hz (temp source=%s, disk=%s)",
      publish_rate_hz_, temperature_source_.c_str(), disk_mount_point_.c_str());
    if (!hardware_model_.empty() || !os_pretty_name_.empty()) {
      RCLCPP_INFO(
        get_logger(), "host: %s — %s (kernel %s)",
        hardware_model_.empty() ? "(unknown hardware)" : hardware_model_.c_str(),
        os_pretty_name_.empty() ? "(unknown OS)" : os_pretty_name_.c_str(),
        kernel_release_.empty() ? "?" : kernel_release_.c_str());
    }
  }

private:
  rclcpp::Publisher<cmeresearch_msgs::msg::SystemStats>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  long prev_idle_ = 0;
  long prev_total_ = 0;
  double publish_rate_hz_ = 0.2;
  std::string temperature_source_ = "auto";
  std::string disk_mount_point_ = "/";
  std::string os_pretty_name_;
  std::string kernel_release_;
  std::string hardware_model_;

  static constexpr uint32_t THROTTLE_ACTIVE_MASK = 0x1u | 0x2u | 0x4u | 0x8u;

  std::optional<double> read_temperature() const
  {
    if (temperature_source_ == "sysfs") {
      return read_temp_sysfs();
    }
    if (temperature_source_ == "vcgencmd") {
      return read_temp_vcgencmd();
    }
    // auto
    if (auto t = read_temp_vcgencmd()) {return t;}
    return read_temp_sysfs();
  }

  void publish_stats()
  {
    cmeresearch_msgs::msg::SystemStats msg;
    msg.header.stamp = now();
    msg.header.frame_id = "robot";

    // CPU
    long idle = 0, total = 0;
    msg.cpu_percent = std::nanf("");
    if (read_cpu_jiffies(idle, total)) {
      const long d_idle = idle - prev_idle_;
      const long d_total = total - prev_total_;
      if (d_total > 0) {
        msg.cpu_percent = static_cast<float>(
          100.0 * (1.0 - static_cast<double>(d_idle) / d_total));
      }
      prev_idle_ = idle;
      prev_total_ = total;
    }

    // Memory
    long mem_total_kb = 0, mem_avail_kb = 0;
    msg.memory_total_mb = 0;
    msg.memory_used_mb = 0;
    msg.memory_percent = std::nanf("");
    if (read_meminfo(mem_total_kb, mem_avail_kb) && mem_total_kb > 0) {
      msg.memory_total_mb = static_cast<uint32_t>(mem_total_kb / 1024);
      msg.memory_used_mb = static_cast<uint32_t>(
        (mem_total_kb - mem_avail_kb) / 1024);
      msg.memory_percent = static_cast<float>(
        100.0 * (1.0 - static_cast<double>(mem_avail_kb) / mem_total_kb));
    }

    // Temperature
    msg.temperature_c = std::nanf("");
    if (auto t = read_temperature()) {
      msg.temperature_c = static_cast<float>(*t);
    }

    // Disk usage on the configured mount point
    msg.disk_total_mb = 0;
    msg.disk_used_mb = 0;
    msg.disk_percent = std::nanf("");
    uint64_t disk_total = 0, disk_used = 0;
    if (read_disk_usage(disk_mount_point_, disk_total, disk_used) && disk_total > 0) {
      constexpr uint64_t MB = 1024ULL * 1024ULL;
      msg.disk_total_mb = static_cast<uint32_t>(disk_total / MB);
      msg.disk_used_mb = static_cast<uint32_t>(disk_used / MB);
      msg.disk_percent = static_cast<float>(
        100.0 * static_cast<double>(disk_used) / static_cast<double>(disk_total));
    }

    // Host identification (cached at startup)
    msg.os_pretty_name = os_pretty_name_;
    msg.kernel_release = kernel_release_;
    msg.hardware_model = hardware_model_;

    // Throttle (RPi-only; zero on non-Pi hosts)
    msg.throttle_bits = 0;
    msg.throttle_active = false;
    if (auto bits = read_throttled_bits()) {
      msg.throttle_bits = *bits;
      msg.throttle_active = (*bits & THROTTLE_ACTIVE_MASK) != 0;
      if (msg.throttle_active) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 60'000,
          "Pi throttled: bits=0x%x (under_voltage=%d cap=%d throttled=%d soft_temp=%d)",
          msg.throttle_bits,
          (msg.throttle_bits & 0x1u) != 0,
          (msg.throttle_bits & 0x2u) != 0,
          (msg.throttle_bits & 0x4u) != 0,
          (msg.throttle_bits & 0x8u) != 0);
      }
    }

    pub_->publish(msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemStatsNode>());
  rclcpp::shutdown();
  return 0;
}
