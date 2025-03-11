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
    // If the packet doesn't start with the correct start character, don't process it.
    if (str.at(0) != '!') {
        return data;
    }
    if (str.at(1) == 'y') {  // This is a Yaw/Pitch/Roll/Compass Message
        // Drop first 2 characters from string, and remove message checksum and line termination
        std::string data_str = str.substr(2, str.size() - 6);
        std::vector<std::string> tokens;
        boost::split(tokens, data_str, boost::is_any_of(" "), boost::token_compress_on);
        if (tokens.size() != 4) {
            return data;
        }
        data.packet_type = PacketType::AHRS;
        data.yaw_deg = atof(tokens.at(0).c_str());
        data.pitch_deg = atof(tokens.at(1).c_str());
        data.roll_deg = atof(tokens.at(2).c_str());
        data.compass_deg = atof(tokens.at(3).c_str());
        data.parsed_ok = true;
    }
    else {
        return data;
    }

    return data;
}
}  // namespace fast_sensors