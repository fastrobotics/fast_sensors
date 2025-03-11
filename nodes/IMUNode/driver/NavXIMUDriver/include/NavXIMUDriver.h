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

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

#include "BaseIMUDriver.h"

//! fast_sensors Namespace
namespace fast_sensors {
/**
 * @brief NavXIMUDriver Class
 * @details
 */
class NavXIMUDriver : public BaseIMUDriver
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
    bool init(eros::eros_diagnostic::Diagnostic diagnostic, eros::Logger* logger) override;
    eros::eros_diagnostic::Diagnostic update(double current_time_sec, double dt) override;

    bool set_comm_device(std::string comm_device, int speed);
    /**
     * @brief Finish and Close Driver
     *
     * @return true
     * @return false
     */
    bool finish() override;
    std::string pretty(std::string mode = "") override;
    int readFromSerialPort(char* buffer, size_t size);

   private:
    std::string comm_device_;
    int fd;

};  // namespace fast_sensors
}  // namespace fast_sensors