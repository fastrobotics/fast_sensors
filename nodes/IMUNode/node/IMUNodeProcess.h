/**
 * @file IMUNodeProcess.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-03-11
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <eros/BaseNodeProcess.h>
#include <eros_diagnostic/Diagnostic.h>
#include <geometry_msgs/PoseStamped.h>

#include "IIMUDriver.h"

namespace fast_sensors {

class IMUNodeProcess : public eros::BaseNodeProcess
{
   public:
    IMUNodeProcess();
    ~IMUNodeProcess();
    eros::eros_diagnostic::Diagnostic finish_initialization();
    void reset();
    eros::eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(eros::command msg);
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();
    void cleanup() {
        base_cleanup();
        return;
    }
    std::string pretty() override;
    sensor_msgs::Imu get_imu_data() {
        return driver->get_imu_data();
    }

   private:
    IIMUDriver* driver;
};
}  // namespace fast_sensors
