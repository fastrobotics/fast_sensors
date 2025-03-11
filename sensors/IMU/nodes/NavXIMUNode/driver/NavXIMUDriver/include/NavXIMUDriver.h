/**
 * @file NavXIMUDriver.h
 * @author David Gitz
 * @brief
 * @date 2025-02-21
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <eros/Logger.h>
#include <eros_utility/ConvertUtility.h>
#include <eros_utility/PrettyUtility.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

//! fast_sensors Namespace
namespace fast_sensors {
/**
 * @brief NavXIMUDriver Class
 * @details
 */
class NavXIMUDriver
{
   public:
    NavXIMUDriver();
    virtual ~NavXIMUDriver();
    /**
     * @brief Initialize GPS Hat Driver
     *
     * @param logger
     * @return true
     * @return false
     */
    bool init(eros::eros_diagnostic::Diagnostic diagnostic, eros::Logger* logger);
    eros::eros_diagnostic::Diagnostic update(double current_time_sec, double dt);

    bool set_comm_device(std::string comm_device, int speed);
    /**
     * @brief Finish and Close Driver
     *
     * @return true
     * @return false
     */
    bool finish();
    std::string pretty(std::string mode = "");
    int readFromSerialPort(char* buffer, size_t size);

   private:
    eros::Logger* logger;
    eros::eros_diagnostic::Diagnostic diagnostic;
    bool fully_initialized{false};
    std::string comm_device_;
    int fd;

};  // namespace fast_sensors
}  // namespace fast_sensors