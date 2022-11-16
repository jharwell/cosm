/**
 * \file env_sensor_impl.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/cosm.hpp"

#include "cosm/hal/sensors/config/env_sensor_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_sensor_impl
 * \ingroup hal sensors
 *
 * \brief Interference describing the necessary functionality for robot sensors
 * to implement to be used with \ref env_sensor.
 */
class env_sensor_impl {
 public:
  env_sensor_impl(void) = default;
  virtual ~env_sensor_impl(void) = default;

  /* Not copy constructable/assignable by default */
  env_sensor_impl(const env_sensor_impl&) = delete;
  env_sensor_impl& operator=(const env_sensor_impl&) = delete;
  env_sensor_impl(env_sensor_impl&&) = default;
  env_sensor_impl& operator=(env_sensor_impl&&) = default;

  /**
   * \brief Detect the environmental feature represented by \p name.
   */
  virtual bool detect(const std::string& name,
                      const chsensors::config::env_sensor_detection_config* config) = 0;
};

NS_END(sensors, hal, cosm);
