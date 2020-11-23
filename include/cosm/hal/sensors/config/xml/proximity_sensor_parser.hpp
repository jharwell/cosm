/**
 * \file proximity_sensor_parser.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_HAL_SENSORS_CONFIG_XML_PROXIMITY_SENSOR_PARSER_HPP_
#define INCLUDE_COSM_HAL_SENSORS_CONFIG_XML_PROXIMITY_SENSOR_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "cosm/hal/sensors/config/proximity_sensor_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, sensors, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class proximity_sensor_parser
 * \ingroup hal sensors config xml
 *
 * \brief Parses XML parameters relating to HAL proximity sensor into \ref
 * proximity_sensor_config.
 */
class proximity_sensor_parser : public rconfig::xml::xml_config_parser {
 public:
  using config_type = proximity_sensor_config;

  ~proximity_sensor_parser(void) override = default;

  /**
   * \brief The root tag that all robot proximity sensor parameters should lie
   * under in the XML tree.
   */
  static constexpr const char kXMLRoot[] = "proximity_sensor";

  bool validate(void) const override RCPPSW_PURE;
  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(xml, config, sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORS_CONFIG_XML_PROXIMITY_SENSOR_PARSER_HPP_ */
