/**
 * @file BaseIMUDriver.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-03-11
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "IIMUDriver.h"
#pragma once
namespace fast_sensors {
class BaseIMUDriver : public IIMUDriver
{
    BaseIMUDriver() {
    }
    virtual ~BaseIMUDriver() {
    }
    bool init(eros::eros_diagnostic::Diagnostic diagnostic, eros::Logger* logger);
    bool is_fully_initialized() {
        return fully_initialized;
    }

    eros::eros_diagnostic::Diagnostic update(double current_time_sec, double dt);

    std::string pretty(std::string mode);

   protected:
    eros::eros_diagnostic::Diagnostic diagnostic;
    eros::Logger* logger;
    bool fully_initialized{false};

   private:
    double prev_current_time_sec{-1.0};
    double run_time{0.0};
};
}  // namespace fast_sensors
