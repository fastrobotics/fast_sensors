
#include "IMUNodeProcess.h"

#include "NavXIMUDriver.h"
namespace fast_sensors {
IMUNodeProcess::IMUNodeProcess() {
}
IMUNodeProcess::~IMUNodeProcess() {
    delete driver;
    delete logger;
}
eros::eros_diagnostic::Diagnostic IMUNodeProcess::finish_initialization() {
    eros::eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    driver = new NavXIMUDriver();
    driver->init(diag, logger);
    if (driver->set_comm_device("/dev/ttyACM0", B115200) == false) {
        logger->log_error("Error Initializing Driver.  Exiting.");
        diag.type = eros::eros_diagnostic::DiagnosticType::SOFTWARE;
        diag.level = eros::Level::Type::ERROR;
        diag.message = eros::eros_diagnostic::Message::INITIALIZING_ERROR;
        diag.description = "Unable to IMU";
        return diag;
    }
    return diag;
}

void IMUNodeProcess::reset() {
}
eros::eros_diagnostic::Diagnostic IMUNodeProcess::update(double t_dt, double t_ros_time) {
    eros::eros_diagnostic::Diagnostic diag = base_update(t_dt, t_ros_time);
    diag = driver->update(t_ros_time, t_dt);
    return diag;
}
std::vector<eros::eros_diagnostic::Diagnostic> IMUNodeProcess::new_commandmsg(eros::command msg) {
    (void)msg;
    std::vector<eros::eros_diagnostic::Diagnostic> diag_list;
    logger->log_warn("No Command Messages Supported at this time.");
    return diag_list;
}
std::vector<eros::eros_diagnostic::Diagnostic> IMUNodeProcess::check_programvariables() {
    std::vector<eros::eros_diagnostic::Diagnostic> diag_list;
    logger->log_warn("No Program Variables Checked.");
    return diag_list;
}
std::string IMUNodeProcess::pretty() {
    std::string str = "Node State: " + eros::Node::NodeStateString(get_nodestate()) + "\n";
    str += driver->pretty();
    return str;
}
}  // namespace fast_sensors