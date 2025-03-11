#include <chrono>
#include <csignal>

#include "NavXIMUDriver.h"
using namespace fast_sensors;
NavXIMUDriver driver;
void printHelp() {
    printf("Tester for NavX IMU Node Driver\n");
    printf("-h This Menu.\n");
    printf("-d Device.  Default: /dev/ttyACM0\n");
    printf("-l Logger Threshold. [DEBUG,INFO,NOTICE,WARN,ERROR]\n");
}
void signalinterrupt_handler(int sig) {
    printf("Killing NavXIMU Driver with Signal: %d\n", sig);
    driver.finish();
    exit(0);
}
int main(int argc, char* argv[]) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    std::string logger_threshold = "DEBUG";
    std::string device = "/dev/ttyUSB0";
    for (;;) {
        switch (getopt(argc,
                       argv,
                       "d:l:h"))  // note the colon (:) to indicate that 'b' has a parameter and
                                  // is not a switch
        {
            case 'd': device = optarg; continue;
            case 'l': logger_threshold = optarg; break;
            case '?': printHelp(); return 0;
            case 'h': printHelp(); return 0;
            default: printHelp(); return 0;
        }

        break;
    }
    eros::Logger* logger = new eros::Logger("DEBUG", "exec_NavXIMUDriver");
    eros::eros_diagnostic::Diagnostic diag =
        eros::eros_diagnostic::Diagnostic("UnitTest",
                                          "UnitTest",
                                          eros::System::MainSystem::SIMROVER,
                                          eros::System::SubSystem::ENTIRE_SYSTEM,
                                          eros::System::Component::NAVIGATION,
                                          eros::eros_diagnostic::DiagnosticType::SENSORS,
                                          eros::eros_diagnostic::Message::INITIALIZING,
                                          eros::Level::Type::INFO,
                                          "Initializing");
    logger->log_debug("Starting NavXIMU Driver");
    driver.init(diag, logger);
    if (driver.set_comm_device(device, B115200) == false) {
        logger->log_error("Error Initializing Driver.  Exiting.");
        return 1;
    }
    double delta_time_sec = 0.01;
    while (true) {
        auto current_time = std::chrono::system_clock::now();
        auto duration_in_seconds = std::chrono::duration<double>(current_time.time_since_epoch());

        double current_time_sec = duration_in_seconds.count();
        driver.update(current_time_sec, delta_time_sec);
        usleep(delta_time_sec * 1000000);

        // logger->log_debug(driver.pretty());
    }

    logger->log_debug("NavXIMU Driver Finished.");
    delete logger;
    return 0;
}