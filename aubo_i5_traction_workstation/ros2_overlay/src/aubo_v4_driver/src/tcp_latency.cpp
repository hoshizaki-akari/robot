#include "aubo_v4_driver/tcp_latency.hpp"

#include <arpa/inet.h>
#include <dirent.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <algorithm>
#include <cstdlib>
#include <limits>

namespace aubo_v4_driver
{
namespace
{
bool matches(int fd, const std::string & host, int port)
{
  in_addr expected{};
  if (port <= 0 || port > 65535 || inet_pton(AF_INET, host.c_str(), &expected) != 1) {
    return false;
  }
  sockaddr_in peer{};
  socklen_t size = sizeof(peer);
  int type = 0, protocol = 0;
  socklen_t option_size = sizeof(type);
  if (getsockopt(fd, SOL_SOCKET, SO_TYPE, &type, &option_size) != 0 || type != SOCK_STREAM) {
    return false;
  }
  option_size = sizeof(protocol);
  return getsockopt(fd, SOL_SOCKET, SO_PROTOCOL, &protocol, &option_size) == 0 &&
         protocol == IPPROTO_TCP &&
         getpeername(fd, reinterpret_cast<sockaddr *>(&peer), &size) == 0 &&
         size == sizeof(peer) && peer.sin_family == AF_INET &&
         peer.sin_addr.s_addr == expected.s_addr && ntohs(peer.sin_port) == port;
}
}

std::vector<int> find_tcp_peers(const std::string & host, int port)
{
  std::vector<int> result;
  DIR * directory = opendir("/proc/self/fd");
  if (!directory) {return result;}
  while (auto * entry = readdir(directory)) {
    char * end = nullptr;
    const long value = std::strtol(entry->d_name, &end, 10);
    if (end == entry->d_name || *end != '\0' || value < 0 ||
      value > std::numeric_limits<int>::max()) {continue;}
    const int fd = static_cast<int>(value);
    if (matches(fd, host, port)) {result.push_back(fd);}
  }
  closedir(directory);
  return result;
}

TcpLatencySample tune_tcp_peers(
  const std::vector<int> & descriptors, const std::string & host, int port)
{
  TcpLatencySample sample;
  for (int fd : descriptors) {
    if (!matches(fd, host, port)) {continue;}
    ++sample.matched;
    const int one = 1;
    int actual = 0;
    socklen_t size = sizeof(actual);
    if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) == 0 &&
      getsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &actual, &size) == 0 && actual == 1)
    {
      ++sample.nodelay;
    }
    if (setsockopt(fd, IPPROTO_TCP, TCP_QUICKACK, &one, sizeof(one)) == 0) {++sample.quickack;}
    tcp_info info{};
    size = sizeof(info);
    if (getsockopt(fd, IPPROTO_TCP, TCP_INFO, &info, &size) == 0) {
      sample.maximum_rtt_s = std::max(sample.maximum_rtt_s, info.tcpi_rtt * 1e-6);
      sample.retransmissions += info.tcpi_total_retrans;
      sample.unacked += info.tcpi_unacked;
    }
  }
  return sample;
}
}  // namespace aubo_v4_driver
