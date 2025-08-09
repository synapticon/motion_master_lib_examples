#include <comm/spoe.h>
#include <core/timer.h>

#include <cassert>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include "loguru.h"

const std::string kIp = "192.168.100.8";
const int kPort = 8080;

int main() {
  auto device = mm::comm::spoe::Device{kIp, kPort};
  LOG_F(INFO, "Connecting to %s:%d", kIp.c_str(), kPort);
  auto connected = device.connect();
  LOG_F(INFO, "done.");

  LOG_F(INFO, "Setting device state to OPERATIONAL (8)");
  device.setState(8);
  LOG_F(INFO, "done.");

  // Check if the socket is now connected
  assert(connected == true);
  assert(device.isConnected() == true);

  LOG_F(INFO, "Getting state...");
  auto state = device.getState();
  LOG_F(INFO, "done. State: %d", state);

  // Expecting state to be OP (0x08)
  assert(state == 0x08);

  {
    LOG_F(INFO, "Loading parameters and reading their values...");
    device.loadParameters(true);
    LOG_F(INFO, "done. Number of parameters: %zu", device.parameters().size());

    auto productCode =
        device.findParameter(0x1018, 0x02).getValue<std::uint32_t>();
    LOG_F(INFO, "Product Code: 0x%04X", productCode);

    auto manufacturerSoftwareVersion =
        device.findParameter(0x100A, 0x00).getValue<std::string>();
    LOG_F(INFO, "Manufacturer Software Version: %s",
          manufacturerSoftwareVersion.c_str());
  }

  {
    LOG_F(INFO, "Reading .hardware_description file...");
    auto data =
        device.readFile(".hardware_description", std::chrono::seconds(5));
    auto text = std::string(data.begin(), data.end());
    LOG_F(INFO, "done. Text:\n%s", text.c_str());
  }

  {
    LOG_F(INFO, "Setting PDO mode to CONTROL...");
    auto modeSet = device.setPdoMode(mm::comm::spoe::PdoMode::CONTROL);
    LOG_F(INFO, "done. Mode set: %s", modeSet ? "true" : "false");

    auto& timestamp = device.findParameter(0x20F0, 0x00);

    LOG_F(INFO, "Timestamp parameter value before exchange: %u",
          timestamp.getValue<uint32_t>());
    LOG_F(INFO, "Core temperature SDO value before exchange: %d",
          mm::core::util::toInteger<int32_t>(device.readSdo(0x2030, 0x01)));
    LOG_F(INFO, "Drive temperature SDO value before exchange: %d",
          mm::core::util::toInteger<int32_t>(device.readSdo(0x2031, 0x01)));

    const int kExchangeCount = 32;
    const int kSleepDurationMs = 1;

    LOG_F(INFO,
          "Exchanging process data %d times with %d ms sleep between "
          "exchanges...",
          kExchangeCount, kSleepDurationMs);

    for (int i = 0; i < kExchangeCount; ++i) {
      device.exchangeProcessDataAndUpdateParameters();
      std::this_thread::sleep_for(std::chrono::milliseconds(kSleepDurationMs));
    }

    LOG_F(INFO, "done.");

    LOG_F(INFO, "Timestamp parameter value after exchange: %u",
          timestamp.getValue<uint32_t>());
    LOG_F(INFO, "Core temperature SDO value after exchange: %d",
          mm::core::util::toInteger<int32_t>(device.readSdo(0x2030, 0x01)));
    LOG_F(INFO, "Drive temperature SDO value after exchange: %d",
          mm::core::util::toInteger<int32_t>(device.readSdo(0x2031, 0x01)));
  }

  LOG_F(INFO, "Disconnecting from %s:%d...", kIp.c_str(), kPort);
  device.disconnect();
  LOG_F(INFO, "done.");
}
