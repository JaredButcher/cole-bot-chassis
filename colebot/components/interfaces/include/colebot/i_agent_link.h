// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>

#include "colebot/i_network_interface.h"

namespace cb {

// The micro-ROS session over the current default interface (PLAN.md §5.2).
// Calls may block for up to their timeout. Ros task only.
class IAgentLink {
 public:
  virtual ~IAgentLink() = default;
  // Looks for an agent: discovery first, then the saved address for this
  // interface, if any. Remembers the agent it found for open().
  virtual bool probe(NetIface iface, uint32_t timeout_ms) = 0;
  // Opens the session to the agent found by probe() and creates the ROS entities.
  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool ping(uint32_t timeout_ms) = 0;
};

}  // namespace cb
