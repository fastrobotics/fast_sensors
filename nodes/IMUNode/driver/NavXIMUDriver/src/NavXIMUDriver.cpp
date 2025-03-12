#include "NavXIMUDriver.h"
namespace fast_sensors {
NavXIMUDriver::NavXIMUDriver() {
}
NavXIMUDriver::~NavXIMUDriver() {
    finish();
}
bool NavXIMUDriver::finish() {
    close(fd);
    return true;
}
bool NavXIMUDriver::init(eros::eros_diagnostic::Diagnostic diagnostic_, eros::Logger* logger_) {
    return BaseIMUDriver::init(diagnostic_, logger_);
}
eros::eros_diagnostic::Diagnostic NavXIMUDriver::update(double current_time_sec, double dt) {
    auto diag = BaseIMUDriver::update(current_time_sec, dt);
    if (diag.level >= eros::Level::Type::ERROR) {
        logger->log_diagnostic(diag);
        return diag;
    }
    char buffer[100];

    int n = readFromSerialPort(buffer, sizeof(buffer) - 1);
    if (n < 0) {
        diag.type = eros::eros_diagnostic::DiagnosticType::COMMUNICATIONS;
        diag.level = eros::Level::Type::ERROR;
        diag.message = eros::eros_diagnostic::Message::DROPPING_PACKETS;
        diag.description = strerror(errno);
        logger->log_diagnostic(diag);
        diagnostic = diag;
        return diag;
    }
    else {
        if (diag.level >= eros::Level::Type::WARN) {
            diagnostic = diag;
            logger->log_diagnostic(diag);
            return diag;
        }
        NavXIMUPacketParser::ParsedPacket packet = NavXIMUPacketParser::parsePacket(buffer);
        if (packet.parsed_ok == true) {
            good_packet_count++;
            packet.time_stamp = ros::Time::now();
            latest_packet = packet;
            diag.type = eros::eros_diagnostic::DiagnosticType::SOFTWARE;
            diag.level = eros::Level::Type::INFO;
            diag.message = eros::eros_diagnostic::Message::NOERROR;
            diag.description = "Driver OK";
        }
        else {
            bad_packet_count++;
            diag.type = eros::eros_diagnostic::DiagnosticType::SOFTWARE;
            diag.level = eros::Level::Type::WARN;
            diag.message = eros::eros_diagnostic::Message::DROPPING_PACKETS;
            logger->log_debug("Unable to parse Packet: " + std::string(buffer, n));
        }
        diagnostic = diag;
        return diag;
    }
}
std::string NavXIMUDriver::pretty(std::string mode) {
    std::string str = "NavX IMU Node Driver";
    str += " Comm Device: " + comm_device_ + "\n";
    str += BaseIMUDriver::pretty(mode);
    str += "Good Packets: " + std::to_string(good_packet_count) +
           " Bad Packets: " + std::to_string(bad_packet_count) + "\n";
    return str;
}
bool NavXIMUDriver::set_comm_device(std::string comm_device, int speed) {
    comm_device_ = comm_device;
    fd = open(comm_device.c_str(), O_RDWR);  // | O_NOCTTY | O_SYNC);
    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if (tcgetattr(fd, &tty) != 0) {
        logger->log_error(strerror(errno));
        return false;
    }

    tty.c_cflag &= ~PARENB;  // Clear parity bit, disabling parity (most common)
    tty.c_cflag &=
        ~CSTOPB;  // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;          // Clear all bits that set the data size
    tty.c_cflag |= CS8;             // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;        // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;                    // Disable echo
    tty.c_lflag &= ~ECHOE;                   // Disable erasure
    tty.c_lflag &= ~ECHONL;                  // Disable new-line echo
    tty.c_lflag &= ~ISIG;                    // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                     ICRNL);  // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;  // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON
    // LINUX)

    tty.c_cc[VTIME] =
        10;  // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        logger->log_error(strerror(errno));
        return false;
    }
    // Send Configuration Request. Not Working, investigate during #1466
    if (0) {
        for (int i = 0; i < 50; ++i) {
            unsigned char msg[] = {0x23, 0x70, 0x01, 0x0A, 0x0D};
            printf("send: x%sx\n", msg);
            write(fd, msg, sizeof(msg));
            char buffer[100];

            int n = readFromSerialPort(buffer, sizeof(buffer) - 1);
            printf("%s\n", buffer);
            usleep(0.5 * 1000000.0);
        }
    }
    fully_initialized = true;
    logger->log_notice("Sonar Array Node Driver Fully Initialized.");
    return true;
}
int NavXIMUDriver::readFromSerialPort(char* buffer, size_t size) {
    return read(fd, buffer, size);
}
sensor_msgs::Imu NavXIMUDriver::get_imu_data() {
    sensor_msgs::Imu imu_data;
    imu_data.header.stamp = latest_packet.time_stamp;
    imu_data.header.frame_id = "imu";
    geometry_msgs::QuaternionStamped orientation;

    tf2::Quaternion quat;
    quat.setRPY(latest_packet.roll_deg * M_PI / 180.0,
                latest_packet.pitch_deg * M_PI / 180.0,
                latest_packet.yaw_deg * M_PI / 180.0);
    imu_data.orientation = tf2::toMsg(quat);
    return imu_data;
}
}  // namespace fast_sensors