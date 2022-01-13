/**
 * \file env_sensor_parser.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_COSM_HAL_ROS_SENSORS_CONFIG_XML_ENV_SENSOR_PARSER_HPP_
#define INCLUDE_COSM_HAL_ROS_SENSORS_CONFIG_XML_ENV_SENSOR_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/config/xml/xml_config_parser.hpp"

#include "cosm/hal/ros/sensors/config/env_sensor_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, sensors, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_sensor_parser
 * \ingroup hal ros sensors config xml
 *
 * \brief Parses XML parameters relating to environmental feature detection into \ref
 * env_sensor_config.
 */
class env_sensor_parser final : public rer::client<env_sensor_parser>,
                                public rconfig::xml::xml_config_parser {
 public:
  using config_type = env_sensor_config;

  env_sensor_parser(void) : ER_CLIENT_INIT("cosm.hal.ros.sensors.config.xml.env_sensor_parser") {}

  void parse(const ticpp::Element&) override {}
  std::string xml_root(void) const override { return ""; }
  void detection_add(const std::string& target) { }
 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return nullptr;
  }
};

NS_END(xml, config, sensors, ros, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ROS_SENSORS_CONFIG_XML_ENV_SENSOR_PARSER_HPP_ */
