#include "NavXIMUDriver.h"
using namespace fast_sensors;
NavXIMUDriver driver;
int main() {
    eros::Logger* logger = new eros::Logger("DEBUG", "exec_NavXIMUDriver");
    logger->log_debug("Starting NavXIMU Driver");

    driver.init(logger);
    double delta_time_sec = 0.25;
    while (true) {
        driver.update(delta_time_sec);
        usleep(delta_time_sec * 1000000);
    }

    logger->log_debug("NavXIMU Driver Finished.");
    delete logger;
    return 0;
}