#include "NavXIMUDriver.h"
namespace fast_sensors {
NavXIMUDriver::NavXIMUDriver() {
}
NavXIMUDriver::~NavXIMUDriver() {
    finish();
}
bool NavXIMUDriver::finish() {
    return true;
}
bool NavXIMUDriver::init(eros::Logger* _logger) {
    logger = _logger;

    return true;
}
bool NavXIMUDriver::update(double dt) {
    return true;
}
std::string NavXIMUDriver::pretty() {
    std::string str;
    return str;
}
}  // namespace fast_sensors