// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "colebot/i_agent_link.h"

namespace cb {

class FakeAgentLink : public IAgentLink {
 public:
  bool probe(NetIface iface, uint32_t) override {
    ++probes_;
    found_ = agent_[static_cast<int>(iface)] ? &agent_[static_cast<int>(iface)] : nullptr;
    return found_ != nullptr;
  }
  bool open() override {
    open_ = found_ != nullptr && open_succeeds_;
    return open_;
  }
  void close() override {
    open_ = false;
    ++closes_;
  }
  bool ping(uint32_t) override { return open_ && alive_; }

  void agentOn(NetIface iface, bool present = true) { agent_[static_cast<int>(iface)] = present; }
  void agentAlive(bool alive) { alive_ = alive; }
  void openSucceeds(bool ok) { open_succeeds_ = ok; }

  bool isOpen() const { return open_; }
  int probes() const { return probes_; }
  int closes() const { return closes_; }

 private:
  bool agent_[2] = {false, false};
  bool* found_ = nullptr;
  bool open_ = false;
  bool alive_ = true;
  bool open_succeeds_ = true;
  int probes_ = 0;
  int closes_ = 0;
};

}  // namespace cb
