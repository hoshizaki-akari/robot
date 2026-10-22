#pragma once

#include <string>
#include <vector>

namespace aubo_v4_driver
{
struct TcpLatencySample
{
  unsigned matched{0}, nodelay{0}, quickack{0}, retransmissions{0}, unacked{0};
  double maximum_rtt_s{0.0};
};
// Inspect only this process's already-open, connected TCP sockets. No connect,
// send, recv, shutdown, credential access or global networking changes.
std::vector<int> find_tcp_peers(const std::string & numeric_ipv4, int port);
// Revalidate peer/type on every use (cached descriptors may have been closed).
// QUICKACK is transient and is deliberately rearmed before each stream send.
TcpLatencySample tune_tcp_peers(
  const std::vector<int> & descriptors, const std::string & numeric_ipv4, int port);
}  // namespace aubo_v4_driver
