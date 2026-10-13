// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <atomic>
#include <cstdint>

#include "colebot/i_agent_link.h"
#include "colebot/i_clock.h"
#include "colebot/i_drive_controller.h"
#include "colebot/i_network_interface.h"

namespace cb {

struct ConnectionParams {
  int64_t eth_grace_us = 5000000;  // WiFi agents are refused this long after Ethernet link-up
  int64_t ping_period_us = 200000;
  uint32_t ping_timeout_ms = 50;
  uint32_t max_missed_pings = 3;
  uint32_t probe_timeout_ms = 500;
};

enum class ConnState : uint8_t { kConnecting, kConnected };

struct ConnectionStatus {
  ConnState state = ConnState::kConnecting;
  NetIface iface = NetIface::kEthernet;  // valid while connected
  uint32_t reconnects = 0;
};

// Connecting / connected state machine (PLAN.md §5.2). CONNECTING keeps both
// interfaces up and probes Ethernet, then WiFi, each poll; the first agent wins
// and the other interface is stopped. Losing the session stops the drive, closes
// the session, restarts the stopped interface and goes back to CONNECTING.
// poll() runs on the ros task and may block for a probe or ping timeout.
class ConnectionManager {
 public:
  ConnectionManager(INetworkInterface& ethernet, INetworkInterface& wifi, IAgentLink& link,
                    IDriveController& drive, IClock& clock, const ConnectionParams& params);

  void poll();
  // Drop the session and search again (console `net reconnect`).
  void reconnect();

  ConnectionStatus status() const;
  bool connected() const { return state_.load() == ConnState::kConnected; }

 private:
  INetworkInterface& interfaceFor(NetIface iface);
  bool tryConnect(NetIface iface);
  void disconnect();

  INetworkInterface& ethernet_;
  INetworkInterface& wifi_;
  IAgentLink& link_;
  IDriveController& drive_;
  IClock& clock_;
  const ConnectionParams params_;

  bool started_ = false;
  int64_t eth_link_since_us_ = -1;
  int64_t last_ping_us_ = 0;
  uint32_t missed_pings_ = 0;
  std::atomic<bool> reconnect_requested_{false};
  std::atomic<ConnState> state_{ConnState::kConnecting};
  std::atomic<NetIface> iface_{NetIface::kEthernet};
  std::atomic<uint32_t> reconnects_{0};
};

}  // namespace cb
