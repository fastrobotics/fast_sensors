/*! \file test_NavXIMUDriver.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "../../include/NavXIMUDriver.h"
using namespace eros;
using namespace ros_hats;
using namespace ros_hats;
packet = '!y-029.08 007.36 000.70 181.9U7E3';
TEST(BasicTest, TestDefinitions) {
    for (uint8_t i = 0; i <= (uint8_t)NavXIMUDriver::StatusType::END_OF_LIST; ++i) {
        if ((i == (uint8_t)NavXIMUDriver::StatusType::UNKNOWN) ||
            (i == (uint8_t)NavXIMUDriver::StatusType::END_OF_LIST)) {
            EXPECT_EQ(NavXIMUDriver::StatusTypeString((NavXIMUDriver::StatusType)i), "UNKNOWN");
        }
        else {
            EXPECT_NE(NavXIMUDriver::StatusTypeString((NavXIMUDriver::StatusType)i), "UNKNOWN");
        }
        EXPECT_NE(NavXIMUDriver::get_level((NavXIMUDriver::StatusType)i),
                  eros::Level::Type::UNKNOWN);
        EXPECT_NE(NavXIMUDriver::get_level((NavXIMUDriver::StatusType)i),
                  eros::Level::Type::END_OF_LIST);
    }
    for (uint8_t i = 0; i <= (uint8_t)NavXIMUDriver::FixType::END_OF_LIST; ++i) {
        if ((i == (uint8_t)NavXIMUDriver::FixType::UNKNOWN) ||
            (i == (uint8_t)NavXIMUDriver::FixType::END_OF_LIST)) {
            EXPECT_EQ(NavXIMUDriver::FixTypeString((NavXIMUDriver::FixType)i), "UNKNOWN");
        }
        else {
            EXPECT_NE(NavXIMUDriver::FixTypeString((NavXIMUDriver::FixType)i), "UNKNOWN");
        }
    }
}
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestNavXIMUDriver");
    NavXIMUDriver SUT;
    SUT.init(logger);

    double timeToRun = 10.0;
    double dt = 0.1;
    double timer = 0.0;
    while (timer <= timeToRun) {
        EXPECT_TRUE(SUT.update(dt));
        SUT.get_gps_data();

        logger->log_debug(SUT.pretty());
        timer += dt;
    }

    // delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
