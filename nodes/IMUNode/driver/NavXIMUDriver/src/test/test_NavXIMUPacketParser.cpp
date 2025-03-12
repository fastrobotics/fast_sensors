
/**
 * @file test_NavXIMUPacketParser.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-03-11
 *
 * @copyright Copyright (c) 2025
 *
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "../../include/NavXIMUPacketParser.h"
using namespace fast_sensors;
//
TEST(BasicTest, ParseMessage_AHRS) {
    {  // Normal Packet
        std::string packet = "!y-029.08 007.36 000.70 181.9U7E3";
        auto data = NavXIMUPacketParser::parsePacket(packet);
        EXPECT_TRUE(data.parsed_ok);
    }
    {  // Normal Packet
        std::string packet = "!y 029.08-007.36 000.70 181.9U7E3";
        auto data = NavXIMUPacketParser::parsePacket(packet);
        EXPECT_TRUE(data.parsed_ok);
    }
    {  // Normal Packet
        std::string packet = "!y 029.08 007.36-000.70 181.9U7E3";
        auto data = NavXIMUPacketParser::parsePacket(packet);
        EXPECT_TRUE(data.parsed_ok);
    }
    {  // Normal Packet
        std::string packet = "!y 029.08 007.36 000.70 181.9U7E3";
        auto data = NavXIMUPacketParser::parsePacket(packet);
        EXPECT_TRUE(data.parsed_ok);
    }
    {  // Normal Packet
        std::string packet = "!y-029.08-007.36-000.70 181.9U7E3";
        auto data = NavXIMUPacketParser::parsePacket(packet);
        EXPECT_TRUE(data.parsed_ok);
    }
    {  // Packet with missing data at start, but then complete.
        std::string packet = " 181.9U7E3!y-029.08-007.36-000.70 181.9U7E3";
        auto data = NavXIMUPacketParser::parsePacket(packet);
        EXPECT_TRUE(data.parsed_ok);
    }
    {  // Packet that doesn't have checksum
        std::string packet = "!y 095.67 007.36 000.69 179.2";
        auto data = NavXIMUPacketParser::parsePacket(packet);
        EXPECT_TRUE(data.parsed_ok);
    }
}
TEST(BasicTest, ParseMessage_MalformedPackets) {
    {  // Empty String
        std::string packet = "";
        auto data = NavXIMUPacketParser::parsePacket(packet);
        EXPECT_FALSE(data.parsed_ok);
    }
    {  // No start character
        std::string packet = "y-029.08 007.36 000.70 181.9U7E3";
        auto data = NavXIMUPacketParser::parsePacket(packet);
        EXPECT_FALSE(data.parsed_ok);
    }
    {  // Incorrect Packet Type
        std::string packet = "!g-029.08 007.36 000.70 181.9U7E3";
        auto data = NavXIMUPacketParser::parsePacket(packet);
        EXPECT_FALSE(data.parsed_ok);
    }
    {  // Incomplete Packet
        std::string packet = "!y-029.08 0";
        auto data = NavXIMUPacketParser::parsePacket(packet);
        EXPECT_FALSE(data.parsed_ok);
    }
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
