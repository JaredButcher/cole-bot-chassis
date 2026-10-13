// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "colebot/i_network_interface.h"

namespace cb {

// A stopped interface has no link and no IP, like the real drivers.
class FakeNetworkInterface : public INetworkInterface {
 public:
  void start() override {
    started_ = true;
    ++starts_;
  }
  void stop() override {
    started_ = false;
    ++stops_;
  }
  bool linkUp() const override { return started_ && link_; }
  bool hasIp() const override { return started_ && link_ && ip_; }
  void makeDefault() override { ++make_default_calls_; }

  void setLink(bool link) { link_ = link; }
  void setIp(bool ip) { ip_ = ip; }
  void connectFully() { link_ = ip_ = true; }

  bool started() const { return started_; }
  int starts() const { return starts_; }
  int stops() const { return stops_; }

 private:
  bool started_ = false;
  bool link_ = false;
  bool ip_ = false;
  int starts_ = 0;
  int stops_ = 0;
  int make_default_calls_ = 0;
};

}  // namespace cb
