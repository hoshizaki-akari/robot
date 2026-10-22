#include <gtest/gtest.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include "aubo_v4_driver/tcp_latency.hpp"

namespace av = aubo_v4_driver;
namespace
{
// Loopback TCP sockets only: no SDK construction or robot connection.
class Loopback : public ::testing::Test
{
protected:
  int listener{-1}, client{-1}, accepted{-1};
  int port{0};
  void SetUp() override
  {
    listener = socket(AF_INET, SOCK_STREAM, 0);
    ASSERT_GE(listener, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ASSERT_EQ(bind(listener, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
    socklen_t size = sizeof(address);
    ASSERT_EQ(getsockname(listener, reinterpret_cast<sockaddr *>(&address), &size), 0);
    port = ntohs(address.sin_port);
    ASSERT_EQ(listen(listener, 1), 0);
    client = socket(AF_INET, SOCK_STREAM, 0);
    ASSERT_GE(client, 0);
    ASSERT_EQ(connect(client, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
    accepted = accept(listener, nullptr, nullptr);
    ASSERT_GE(accepted, 0);
  }
  void TearDown() override
  {
    if (accepted >= 0) {close(accepted);}
    if (client >= 0) {close(client);}
    if (listener >= 0) {close(listener);}
  }
};
}

TEST_F(Loopback, OnlyExactExistingTcpPeerGetsNodelayAndQuickack)
{
  const auto found = av::find_tcp_peers("127.0.0.1", port);
  EXPECT_NE(std::find(found.begin(), found.end(), client), found.end());
  const auto tuned = av::tune_tcp_peers(found, "127.0.0.1", port);
  EXPECT_EQ(tuned.matched, 1U);
  EXPECT_EQ(tuned.nodelay, 1U);
  EXPECT_EQ(tuned.quickack, 1U);
  int actual = 0;
  socklen_t size = sizeof(actual);
  ASSERT_EQ(getsockopt(client, IPPROTO_TCP, TCP_NODELAY, &actual, &size), 0);
  EXPECT_EQ(actual, 1);
  EXPECT_EQ(av::tune_tcp_peers(found, "127.0.0.1", port).nodelay, 1U);
}

TEST_F(Loopback, WrongHostPortInvalidClosedOrListeningDescriptorsRemainUntouched)
{
  EXPECT_EQ(av::tune_tcp_peers({client}, "127.0.0.2", port).matched, 0U);
  EXPECT_EQ(av::tune_tcp_peers({client}, "127.0.0.1", 0).matched, 0U);
  EXPECT_EQ(av::tune_tcp_peers({client}, "127.0.0.1", 65536).matched, 0U);
  EXPECT_EQ(av::tune_tcp_peers({client}, "not-an-address", port).matched, 0U);
  EXPECT_EQ(av::tune_tcp_peers({-1, listener}, "127.0.0.1", port).matched, 0U);
  int actual = 1;
  socklen_t size = sizeof(actual);
  ASSERT_EQ(getsockopt(client, IPPROTO_TCP, TCP_NODELAY, &actual, &size), 0);
  EXPECT_EQ(actual, 0);
  const int old = client;
  close(client);
  client = -1;
  EXPECT_EQ(av::tune_tcp_peers({old}, "127.0.0.1", port).matched, 0U);
}
