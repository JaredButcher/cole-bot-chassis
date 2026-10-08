// SPDX-License-Identifier: GPL-3.0-or-later
//
// Board constants for the ESP32-C6-DevKitC-1 chassis controller: pin map, drive
// defaults and ROS names. Plain C++ with no ESP-IDF headers, so core code and
// host tests can use it too. Pins are GPIO numbers.
//
// Avoided pins: 4, 5, 8, 9, 15 (strapping; 8 is the on-board RGB LED),
// 12, 13 (USB-Serial-JTAG), 16, 17 (UART0, wired to the on-board USB-UART bridge).
#pragma once

#include <cstdint>

namespace colebot::board {

// ---- DShot ESCs (one bidirectional pin each, external pull-up) ----
inline constexpr int kDshotLeftGpio = 2;
inline constexpr int kDshotRightGpio = 3;

// ---- Hardware e-stop input (PLAN.md §7.3) ----
// Normally-closed auxiliary contact to GND with a pull-up:
// low = released, high = pressed. An open circuit reads as pressed.
inline constexpr int kEstopGpio = 10;
inline constexpr bool kEstopPressedLevel = true;

// ---- W5500 SPI Ethernet on GP-SPI2 ----
inline constexpr int kEthSclkGpio = 19;
inline constexpr int kEthMosiGpio = 18;
inline constexpr int kEthMisoGpio = 20;
inline constexpr int kEthCsGpio = 21;
inline constexpr int kEthIntGpio = 22;
inline constexpr int kEthRstGpio = 23;

// ---- Reserved for later sprints ----
inline constexpr int kI2cSdaGpio = 6;           // IMU (sprint 3)
inline constexpr int kI2cSclGpio = 7;           // IMU (sprint 3)
inline constexpr int kBatteryVoltageGpio = 0;   // ADC1_CH0 (sprint 2)
inline constexpr int kBatteryCurrentGpio = 1;   // ADC1_CH1 (sprint 2)
inline constexpr int kSpareGpio = 11;

// ---- Drive defaults (runtime parameters, PLAN.md §4.2) ----
inline constexpr uint8_t kDefaultMotorPoles = 14;     // 12N14P outrunner
inline constexpr float kDefaultDriveReduction = 1.0f;  // placeholder until measured
inline constexpr uint32_t kDefaultCmdTimeoutMs = 250;
inline constexpr uint32_t kDefaultStateIdleHz = 10;

// ---- ROS names (must match the URDF in ros/colebot_description) ----
inline constexpr const char* kNodeName = "chassis";
inline constexpr const char* kLeftWheelJoint = "left_wheel_joint";
inline constexpr const char* kRightWheelJoint = "right_wheel_joint";

// ---- Network (PLAN.md §5.1). Ethernet is static; WiFi uses DHCP. ----
inline constexpr uint8_t kDefaultEthIp[4] = {192, 168, 50, 2};
inline constexpr uint8_t kDefaultEthPrefixLen = 24;
inline constexpr uint16_t kDefaultAgentPort = 8888;

}  // namespace colebot::board
