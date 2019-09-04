/**
 * @file ground_sensor_detection_parser.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_HAL_SENSORS_CONFIG_XML_GROUND_SENSOR_DETECTION_PARSER_HPP_
#define INCLUDE_COSM_HAL_SENSORS_CONFIG_XML_GROUND_SENSOR_DETECTION_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>
#include <list>

#include "cosm/hal/sensors/config/ground_sensor_detection_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"
#include "cosm/hal/sensors/config/xml/ground_sensor_detection_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, sensors, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class ground_sensor_detection_parser
 * @ingroup cosm hal sensors config xml
 *
 * @brief Parses XML parameters relating to HAL ground sensor into \ref
 * ground_sensor_detection_config.
 */
class ground_sensor_detection_parser : public rconfig::xml::xml_config_parser {
 public:
  using config_type = ground_sensor_detection_config;

  explicit ground_sensor_detection_parser(const std::string& name)
      : m_name(name) {}

  ~ground_sensor_detection_parser(void) override = default;

  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return m_name; }

  void set_name(const std::string& name) { m_name = name; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::string                  m_name;
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(xml, config, sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORS_CONFIG_XML_GROUND_SENSOR_DETECTION_PARSER_HPP_ */
