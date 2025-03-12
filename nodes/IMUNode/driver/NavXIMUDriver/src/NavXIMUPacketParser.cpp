#include "NavXIMUPacketParser.h"

namespace fast_sensors {
NavXIMUPacketParser::NavXIMUPacketParser() {
}
NavXIMUPacketParser::~NavXIMUPacketParser() {
}
NavXIMUPacketParser::ParsedPacket NavXIMUPacketParser::parsePacket(std::string str) {
    ParsedPacket data;
    // If it's an empty string, don't try anything.
    if (str.size() == 0) {
        return data;
    }
    // If the packet doesn't start with the correct start character
    if (str.at(0) != '!') {
        int res = str.find('!');
        if (res != std::string::npos) {
            str = str.substr(res);
        }
        else {
            return data;
        }
    }
    if (str.at(1) == 'y') {  // This is a Yaw/Pitch/Roll/Compass Message
        if (str.size() < 29) {
            return data;
        }
        // Drop first 2 characters from string, and remove message checksum and line termination
        std::string data_str = str.substr(2, str.size());

        data.packet_type = PacketType::AHRS;
        std::string v1 = data_str.substr(0, 7);
        std::string v2 = data_str.substr(7, 7);
        std::string v3 = data_str.substr(14, 7);
        std::string v4 = data_str.substr(22, 7);

        data.yaw_deg = atof(v1.c_str());
        data.pitch_deg = atof(v2.c_str());
        data.roll_deg = atof(v3.c_str());
        data.compass_deg = atof(v4.c_str());

        data.parsed_ok = true;
    }
    else {
        return data;
    }

    return data;
}
}  // namespace fast_sensors