/**
 * @file NavXIMUPacketParser.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-03-11
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <stdlib.h>

#include <boost/algorithm/string.hpp>
#include <string>
#include <vector>

#include "ros/time.h"
namespace fast_sensors {
class NavXIMUPacketParser
{
   public:
    enum class PacketType { UNKNOWN = 0, AHRS = 1, END_OF_LIST = 2 };
    struct ParsedPacket {
        bool parsed_ok{false};
        ros::Time time_stamp;
        PacketType packet_type;
        double yaw_deg{0.0};
        double pitch_deg{0.0};
        double roll_deg{0.0};
        double compass_deg{0.0};
    };
    NavXIMUPacketParser();
    virtual ~NavXIMUPacketParser();
    static ParsedPacket parsePacket(std::string str);
};
}  // namespace fast_sensors