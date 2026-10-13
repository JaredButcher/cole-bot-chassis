// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>

namespace cb {

enum class NetIface : uint8_t { kEthernet, kWifi };

// One network interface (W5500 Ethernet or WiFi station). Ros task only.
class INetworkInterface {
 public:
  virtual ~INetworkInterface() = default;
  virtual void start() = 0;  // driver up; connects / negotiates in the background
  virtual void stop() = 0;   // driver down (WiFi radio off, W5500 stopped)
  virtual bool linkUp() const = 0;  // cable link (Ethernet) or associated (WiFi)
  virtual bool hasIp() const = 0;
  virtual void makeDefault() = 0;   // route outgoing traffic (and micro-ROS UDP) through it
};

}  // namespace cb
