// SPDX-License-Identifier: GPL-3.0-or-later
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "colebot/connection_manager.h"
#include "fake_agent_link.h"
#include "fake_clock.h"
#include "fake_network_interface.h"
#include "mock_drive_controller.h"

namespace cb {
namespace {

using ::testing::NiceMock;

class ConnectionManagerTest : public ::testing::Test {
 protected:
  // Polls every 100 ms for the given time.
  void pollFor(int64_t ms) {
    for (int64_t t = 0; t < ms; t += 100) {
      manager_.poll();
      clock_.advanceMs(100);
    }
  }

  FakeNetworkInterface eth_;
  FakeNetworkInterface wifi_;
  FakeAgentLink link_;
  NiceMock<MockDriveController> drive_;
  FakeClock clock_;
  ConnectionManager manager_{eth_, wifi_, link_, drive_, clock_, ConnectionParams{}};
};

TEST_F(ConnectionManagerTest, StartsBothInterfacesAndWaits) {
  manager_.poll();
  EXPECT_TRUE(eth_.started());
  EXPECT_TRUE(wifi_.started());
  EXPECT_FALSE(manager_.connected());
  EXPECT_EQ(link_.probes(), 0);  // no IP yet: nothing to probe
}

TEST_F(ConnectionManagerTest, EthernetWinsWhenBothHaveAgents) {
  eth_.connectFully();
  wifi_.connectFully();
  link_.agentOn(NetIface::kEthernet);
  link_.agentOn(NetIface::kWifi);
  manager_.poll();
  EXPECT_TRUE(manager_.connected());
  EXPECT_EQ(manager_.status().iface, NetIface::kEthernet);
  EXPECT_FALSE(wifi_.started());  // the other interface is stopped
  EXPECT_TRUE(eth_.started());
}

TEST_F(ConnectionManagerTest, WifiRefusedDuringEthernetGrace) {
  eth_.setLink(true);  // cable in, but no IP / agent yet
  wifi_.connectFully();
  link_.agentOn(NetIface::kWifi);
  pollFor(4500);
  EXPECT_FALSE(manager_.connected());

  // Ethernet comes up inside the grace period and wins.
  eth_.setIp(true);
  link_.agentOn(NetIface::kEthernet);
  manager_.poll();
  EXPECT_TRUE(manager_.connected());
  EXPECT_EQ(manager_.status().iface, NetIface::kEthernet);
}

TEST_F(ConnectionManagerTest, WifiAcceptedAfterGraceWhenEthernetHasNoAgent) {
  eth_.connectFully();  // link and IP, but nothing answers
  wifi_.connectFully();
  link_.agentOn(NetIface::kWifi);
  pollFor(4900);
  EXPECT_FALSE(manager_.connected());
  pollFor(200);
  EXPECT_TRUE(manager_.connected());
  EXPECT_EQ(manager_.status().iface, NetIface::kWifi);
  EXPECT_FALSE(eth_.started());
}

TEST_F(ConnectionManagerTest, WifiImmediatelyWithoutEthernetCable) {
  wifi_.connectFully();
  link_.agentOn(NetIface::kWifi);
  manager_.poll();
  EXPECT_TRUE(manager_.connected());
  EXPECT_EQ(manager_.status().iface, NetIface::kWifi);
}

TEST_F(ConnectionManagerTest, FailedOpenKeepsConnecting) {
  wifi_.connectFully();
  link_.agentOn(NetIface::kWifi);
  link_.openSucceeds(false);
  pollFor(500);
  EXPECT_FALSE(manager_.connected());
  EXPECT_TRUE(eth_.started());  // nothing was stopped
  link_.openSucceeds(true);
  manager_.poll();
  EXPECT_TRUE(manager_.connected());
}

TEST_F(ConnectionManagerTest, MissedPingsStopDriveAndReconnectOverOtherInterface) {
  eth_.connectFully();
  wifi_.connectFully();
  link_.agentOn(NetIface::kEthernet);
  link_.agentOn(NetIface::kWifi);
  manager_.poll();
  ASSERT_EQ(manager_.status().iface, NetIface::kEthernet);

  link_.agentAlive(false);
  EXPECT_CALL(drive_, stopForSessionLoss()).Times(1);
  pollFor(500);  // pings at +200 and +400 ms both missed: still connected
  EXPECT_TRUE(manager_.connected());
  pollFor(200);  // third miss at +600 ms
  EXPECT_FALSE(manager_.connected());
  EXPECT_EQ(link_.closes(), 1);
  EXPECT_TRUE(wifi_.started());  // restarted for the next search
  EXPECT_EQ(manager_.status().reconnects, 1u);
}

TEST_F(ConnectionManagerTest, EthernetLinkDownDisconnectsImmediately) {
  eth_.connectFully();
  link_.agentOn(NetIface::kEthernet);
  manager_.poll();
  ASSERT_TRUE(manager_.connected());

  EXPECT_CALL(drive_, stopForSessionLoss()).Times(1);
  eth_.setLink(false);
  manager_.poll();
  EXPECT_FALSE(manager_.connected());
  EXPECT_TRUE(wifi_.started());

  // Fails over to WiFi without waiting for the grace period (no cable).
  wifi_.connectFully();
  link_.agentOn(NetIface::kWifi);
  link_.agentAlive(true);
  manager_.poll();
  EXPECT_TRUE(manager_.connected());
  EXPECT_EQ(manager_.status().iface, NetIface::kWifi);
}

TEST_F(ConnectionManagerTest, NoSwitchBackWhileConnectedOverWifi) {
  wifi_.connectFully();
  link_.agentOn(NetIface::kWifi);
  manager_.poll();
  ASSERT_EQ(manager_.status().iface, NetIface::kWifi);

  // Cable plugged in later: Ethernet is stopped, so it stays on WiFi.
  eth_.connectFully();
  link_.agentOn(NetIface::kEthernet);
  pollFor(10000);
  EXPECT_TRUE(manager_.connected());
  EXPECT_EQ(manager_.status().iface, NetIface::kWifi);
  EXPECT_FALSE(eth_.started());
}

TEST_F(ConnectionManagerTest, ReconnectRequestSearchesAgain) {
  wifi_.connectFully();
  link_.agentOn(NetIface::kWifi);
  manager_.poll();
  ASSERT_EQ(manager_.status().iface, NetIface::kWifi);
  eth_.connectFully();
  link_.agentOn(NetIface::kEthernet);

  EXPECT_CALL(drive_, stopForSessionLoss()).Times(1);
  manager_.reconnect();
  manager_.poll();  // drops the session, restarts Ethernet
  EXPECT_FALSE(manager_.connected());
  manager_.poll();  // finds the agent on Ethernet
  EXPECT_TRUE(manager_.connected());
  EXPECT_EQ(manager_.status().iface, NetIface::kEthernet);
}

}  // namespace
}  // namespace cb
