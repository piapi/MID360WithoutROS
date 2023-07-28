#include "comm.h"
#include <string.h>
#include <arpa/inet.h>

namespace livox_mid {

/** Common function --------------------------------------------------------- */
bool IsFilePathValid(const char *path_str) {
  int str_len = strlen(path_str);

  if ((str_len > kPathStrMinSize) && (str_len < kPathStrMaxSize)) {
    return true;
  } else {
    return false;
  }
}

uint32_t CalculatePacketQueueSize(const double publish_freq) {
  uint32_t queue_size = 10;
  if (publish_freq > 10.0) {
    queue_size = static_cast<uint32_t>(publish_freq) + 1;
  }
  return queue_size;
}

std::string IpNumToString(uint32_t ip_num) {
  struct in_addr ip;
  ip.s_addr = ip_num;
  return std::string(inet_ntoa(ip));
}

uint32_t IpStringToNum(std::string ip_string) {
  return static_cast<uint32_t>(inet_addr(ip_string.c_str()));
}

std::string ReplacePeriodByUnderline(std::string str) {
  std::size_t pos = str.find(".");
  while (pos != std::string::npos) {
    str.replace(pos, 1, "_");
    pos = str.find(".");
  }
  return str;
}

} // namespace livox_ros

