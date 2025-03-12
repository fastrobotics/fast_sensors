#include "IMUNode.h"
bool kill_node = false;
namespace fast_sensors {

IMUNode::IMUNode()
    : system_command_action_server(*n.get(),
                                   read_robotnamespace() + "SystemCommandAction",
                                   boost::bind(&IMUNode::system_commandAction_Callback, this, _1),
                                   false) {
    system_command_action_server.start();
}
IMUNode::~IMUNode() {
}
void IMUNode::system_commandAction_Callback(const eros::system_commandGoalConstPtr &goal) {
    eros::eros_diagnostic::Diagnostic diag = process->get_root_diagnostic();
    eros::system_commandResult system_commandResult_;
    system_command_action_server.setAborted(system_commandResult_);
    diag = process->update_diagnostic(
        eros::eros_diagnostic::DiagnosticType::COMMUNICATIONS,
        eros::Level::Type::WARN,
        eros::eros_diagnostic::Message::DROPPING_PACKETS,
        "Received unsupported CommandAction: " +
            eros::Command::CommandString((eros::Command::Type)goal->Command));
    logger->log_diagnostic(diag);
}
void IMUNode::command_Callback(const eros::command::ConstPtr &t_msg) {
    eros::command cmd = eros::eros_utility::ConvertUtility::convert_fromptr(t_msg);
    eros::eros_diagnostic::Diagnostic diag = process->get_root_diagnostic();
    diag = process->update_diagnostic(
        eros::eros_diagnostic::DiagnosticType::COMMUNICATIONS,
        eros::Level::Type::WARN,
        eros::eros_diagnostic::Message::DROPPING_PACKETS,
        "Received unsupported Command: " +
            eros::Command::CommandString((eros::Command::Type)cmd.Command));
    logger->log_diagnostic(diag);
}
bool IMUNode::changenodestate_service(eros::srv_change_nodestate::Request &req,
                                      eros::srv_change_nodestate::Response &res) {
    eros::Node::State req_state = eros::Node::NodeState(req.RequestedNodeState);
    process->request_statechange(req_state);
    res.NodeState = eros::Node::NodeStateString(process->get_nodestate());
    return true;
}
bool IMUNode::start() {
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new IMUNodeProcess();
    set_basenodename(BASE_NODE_NAME);
    initialize_firmware(
        MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
    diagnostic = preinitialize_basenode();
    if (diagnostic.level > eros::Level::Type::WARN) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return false;
        // LCOV_EXCL_STOP
    }
    diagnostic = read_launchparameters();
    if (diagnostic.level > eros::Level::Type::WARN) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return false;
        // LCOV_EXCL_STOP
    }

    process->initialize(get_basenodename(),
                        get_nodename(),
                        get_hostname(),
                        DIAGNOSTIC_SYSTEM,
                        DIAGNOSTIC_SUBSYSTEM,
                        DIAGNOSTIC_COMPONENT,
                        logger);
    std::vector<eros::eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::SOFTWARE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::DATA_STORAGE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::SYSTEM_RESOURCE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::COMMUNICATIONS);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::POSE);
    process->enable_diagnostics(diagnostic_types);
    process->finish_initialization();
    diagnostic = finish_initialization();
    if (diagnostic.level > eros::Level::Type::WARN) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return false;
        // LCOV_EXCL_STOP
    }
    if (diagnostic.level < eros::Level::Type::WARN) {
        diagnostic.type = eros::eros_diagnostic::DiagnosticType::SOFTWARE;
        diagnostic.level = eros::Level::Type::INFO;
        diagnostic.message = eros::eros_diagnostic::Message::NOERROR;
        diagnostic.description = "Node Configured.  Initializing.";
        get_logger()->log_diagnostic(diagnostic);
    }
    if (process->request_statechange(eros::Node::State::RUNNING, true) == false) {
        // No practical way to unit test
        // LCOV_EXCL_START
        logger->log_warn("Unable to Change State to: " +
                         eros::Node::NodeStateString(eros::Node::State::RUNNING));
        // LCOV_EXCL_STOP
    }
    logger->log_notice("Node State: " + eros::Node::NodeStateString(process->get_nodestate()));
    status = true;
    return status;
}
eros::eros_diagnostic::Diagnostic IMUNode::read_launchparameters() {
    eros::eros_diagnostic::Diagnostic diag = diagnostic;
    command_sub = n->subscribe<eros::command>(
        get_robotnamespace() + "SystemCommand", 10, &IMUNode::command_Callback, this);
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
eros::eros_diagnostic::Diagnostic IMUNode::finish_initialization() {
    eros::eros_diagnostic::Diagnostic diag = diagnostic;
    std::string srv_nodestate_topic = "srv_nodestate_change";
    nodestate_srv =
        n->advertiseService(srv_nodestate_topic, &IMUNode::changenodestate_service, this);
    imu_pose_pub = n->advertise<geometry_msgs::PoseStamped>(robot_namespace + "/robot/imu", 20);
    diag = process->update_diagnostic(eros::eros_diagnostic::DiagnosticType::COMMUNICATIONS,
                                      eros::Level::Type::INFO,
                                      eros::eros_diagnostic::Message::NOERROR,
                                      "Comms Ready.");
    diag = process->update_diagnostic(eros::eros_diagnostic::DiagnosticType::SOFTWARE,
                                      eros::Level::Type::INFO,
                                      eros::eros_diagnostic::Message::NOERROR,
                                      "Running");
    diag = process->update_diagnostic(eros::eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                      eros::Level::Type::INFO,
                                      eros::eros_diagnostic::Message::NOERROR,
                                      "All Configuration Files Loaded.");
    diag = process->update_diagnostic(eros::eros_diagnostic::DiagnosticType::POSE,
                                      eros::Level::Type::WARN,
                                      eros::eros_diagnostic::Message::INITIALIZING,
                                      "GPS Initializing...");
    return diag;
}
bool IMUNode::run_loop1() {
    return true;
}
bool IMUNode::run_loop2() {
    return true;
}
bool IMUNode::run_loop3() {
    process->update(0.02, ros::Time::now().toSec());
    imu_pose_pub.publish(process->get_pose());
    return true;
    return true;
}
bool IMUNode::run_001hz() {
    return true;
}
bool IMUNode::run_01hz() {
    return true;
}
bool IMUNode::run_01hz_noisy() {
    eros::eros_diagnostic::Diagnostic diag = diagnostic;
    logger->log_debug(pretty());
    return true;
}
std::string IMUNode::pretty() {
    std::string str = process->pretty();
    return str;
}
bool IMUNode::run_1hz() {
    logger->log_debug(pretty());
    std::vector<eros::eros_diagnostic::Diagnostic> latest_diagnostics =
        process->get_latest_diagnostics();
    for (std::size_t i = 0; i < latest_diagnostics.size(); ++i) {
        logger->log_diagnostic(latest_diagnostics.at(i));
        diagnostic_pub.publish(
            eros::eros_diagnostic::DiagnosticUtility::convert(latest_diagnostics.at(i)));
    }
    eros::eros_diagnostic::Diagnostic diag = process->get_root_diagnostic();
    if (process->get_nodestate() == eros::Node::State::RESET) {
        base_reset();
        process->reset();
        logger->log_notice("Node has Reset");
        if (process->request_statechange(eros::Node::State::RUNNING) == false) {
            // No practical way to unit test
            // LCOV_EXCL_START
            diag = process->update_diagnostic(eros::eros_diagnostic::DiagnosticType::SOFTWARE,
                                              eros::Level::Type::ERROR,
                                              eros::eros_diagnostic::Message::DEVICE_NOT_AVAILABLE,
                                              "Not able to Change Node State to Running.");
            logger->log_diagnostic(diag);
            // LCOV_EXCL_STOP
        }
    }
    return true;
}
bool IMUNode::run_10hz() {
    update_diagnostics(process->get_diagnostics());
    update_ready_to_arm(process->get_ready_to_arm());
    return true;
}
void IMUNode::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
void IMUNode::cleanup() {
    process->request_statechange(eros::Node::State::FINISHED);
    process->cleanup();
    delete process;
    base_cleanup();
}
// No practical way to unit test
}  // namespace fast_sensors
// LCOV_EXCL_START
void signalinterrupt_handler(int sig) {
    printf("Killing IMUNode with Signal: %d\n", sig);
    kill_node = true;
    exit(0);
}
// LCOV_EXCL_STOP

int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    ros::init(argc, argv, "imu_node");
    fast_sensors::IMUNode *node = new fast_sensors::IMUNode();
    bool status = node->start();
    if (status == false) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return EXIT_FAILURE;
        // LCOV_EXCL_STOP
    }
    std::thread thread(&fast_sensors::IMUNode::thread_loop, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }
    node->cleanup();
    thread.detach();
    delete node;
    return 0;
}
