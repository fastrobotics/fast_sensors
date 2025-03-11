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
    bool init(eros::Logger* logger);
    /**
     * @brief Update NavXIMUDriver
     * @details
     *
     * @param dt Delta Time in seconds.
     * @return true
     * @return false
     */
    bool update(double dt);

    /**
     * @brief Finish and Close Driver
     *
     * @return true
     * @return false
     */
    bool finish();

    std::string pretty();

   private:
    eros::Logger* logger;
};
}  // namespace fast_sensors