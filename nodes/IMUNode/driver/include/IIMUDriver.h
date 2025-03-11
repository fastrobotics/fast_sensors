/**
 * @file IIMUDriver.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-03-11
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <eros/Logger.h>
#include <eros_utility/ConvertUtility.h>
#include <eros_utility/PrettyUtility.h>

#pragma once
namespace fast_sensors {
class IIMUDriver
{
   public:
    NavXIMUDriver();
    virtual ~NavXIMUDriver();
    /**
     * @brief Initialize the IMU Driver Driver
     *
     * @param logger
     * @return true
     * @return false
     */
    virtual bool init(eros::eros_diagnostic::Diagnostic diagnostic, eros::Logger* logger) = 0;
    virtual eros::eros_diagnostic::Diagnostic update(double current_time_sec, double dt) = 0;

    virtual bool finish() = 0;
    virtual std::string pretty(std::string mode = "") = 0;
};
}  // namespace fast_sensors