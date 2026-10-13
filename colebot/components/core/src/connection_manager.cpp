// SPDX-License-Identifier: GPL-3.0-or-later
#include "colebot/connection_manager.h"

namespace cb {

namespace {
NetIface other(NetIface iface) {
  return iface == NetIface::kEthernet ? NetIface::kWifi : NetIface::kEthernet;
}
}  // namespace

ConnectionManager::ConnectionManager(INetworkInterface& ethernet, INetworkInterface& wifi,
                                     IAgentLink& link, IDriveController& drive, IClock& clock,
                                     const ConnectionParams& params)
    : ethernet_(ethernet), wifi_(wifi), link_(link), drive_(drive), clock_(clock),
      params_(params) {}

INetworkInterface& ConnectionManager::interfaceFor(NetIface iface) {
  return iface == NetIface::kEthernet ? ethernet_ : wifi_;
}

void ConnectionManager::poll() {
  if (!started_) {
    ethernet_.start();
    wifi_.start();
    started_ = true;
  }
  const int64_t now = clock_.nowUs();
  if (ethernet_.linkUp()) {
    if (eth_link_since_us_ < 0) eth_link_since_us_ = now;
  } else {
    eth_link_since_us_ = -1;
  }

  if (state_.load() == ConnState::kConnected) {
    if (reconnect_requested_.exchange(false)) {
      disconnect();
      return;
    }
    if (iface_.load() == NetIface::kEthernet && !ethernet_.linkUp()) {
      disconnect();
      return;
    }
    if (now - last_ping_us_ >= params_.ping_period_us) {
      last_ping_us_ = now;
      missed_pings_ = link_.ping(params_.ping_timeout_ms) ? 0 : missed_pings_ + 1;
      if (missed_pings_ >= params_.max_missed_pings) disconnect();
    }
    return;
  }

  reconnect_requested_ = false;
  if (ethernet_.hasIp() && tryConnect(NetIface::kEthernet)) return;
  const bool eth_grace = eth_link_since_us_ >= 0 && now - eth_link_since_us_ < params_.eth_grace_us;
  if (wifi_.hasIp() && !eth_grace) tryConnect(NetIface::kWifi);
}

bool ConnectionManager::tryConnect(NetIface iface) {
  INetworkInterface& net = interfaceFor(iface);
  net.makeDefault();
  if (!link_.probe(iface, params_.probe_timeout_ms) || !link_.open()) return false;
  interfaceFor(other(iface)).stop();
  iface_ = iface;
  missed_pings_ = 0;
  last_ping_us_ = clock_.nowUs();
  state_ = ConnState::kConnected;
  return true;
}

void ConnectionManager::disconnect() {
  drive_.stopForSessionLoss();
  link_.close();
  interfaceFor(other(iface_.load())).start();
  state_ = ConnState::kConnecting;
  ++reconnects_;
}

void ConnectionManager::reconnect() { reconnect_requested_ = true; }

ConnectionStatus ConnectionManager::status() const {
  return {state_.load(), iface_.load(), reconnects_.load()};
}

}  // namespace cb
