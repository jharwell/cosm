/**
 * \file env_sensor_config.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>
#include <string>
#include <utility>

#include "rcppsw/config/base_config.hpp"
#include "cosm/hal/sensors/config/env_sensor_detection_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::sensors::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct env_sensor_config
 * \ingroup hal sensors config
 *
 * \brief Configuration for environment sensors, for robots that have them.
 */
struct env_sensor_config : public rconfig::base_config {
  std::map<std::string, env_sensor_detection_config> detect_map{};
};

} /* namespace cosm::hal::sensors::config */
