#include <comm/spoe.h>

#include <cassert>
#include <string>

#include "loguru.h"

const std::string kIp = "192.168.100.8";
const int kPort = 8080;

int main() {
  auto device = mm::comm::spoe::Device{kIp, kPort};
  LOG_F(INFO, "Connecting to %s:%d", kIp.c_str(), kPort);
  device.connect();
  LOG_F(INFO, "done.");
}
