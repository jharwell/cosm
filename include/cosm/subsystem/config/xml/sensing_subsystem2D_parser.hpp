/**
 * \file sensing_subsystem2D_parser.hpp
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

#ifndef INCLUDE_COSM_SUBSYSTEM_CONFIG_XML_SENSING_SUBSYSTEM2D_PARSER_HPP_
#define INCLUDE_COSM_SUBSYSTEM_CONFIG_XML_SENSING_SUBSYSTEM2D_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "cosm/hal/sensors/config/xml/proximity_sensor_parser.hpp"
#include "cosm/hal/sensors/config/xml/ground_sensor_parser.hpp"
#include "cosm/subsystem/config/sensing_subsystem2D_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class sensing_subsystem2D_parser
 * \ingroup subsystem config xml
 *
 * \brief Parses XML parameters relating to sensings into \ref
 * sensing_subsystem2D_config.
 */
class sensing_subsystem2D_parser final : public rconfig::xml::xml_config_parser {
 public:
  using config_type = sensing_subsystem2D_config;

  ~sensing_subsystem2D_parser(void) override = default;

  /**
   * \brief The root tag that all robot 2D sensing subsystem parameters should
   * lie under in the XML tree.
   */
  static constexpr char kXMLRoot[] = "sensing_subsystem2D";

  bool validate(void) const override RCSW_PURE;
  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }

  void ground_detection_add(const std::string& target) {
    m_ground.detection_add(target);
  }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>                       m_config{nullptr};
  hal::sensors::config::xml::proximity_sensor_parser m_proximity{};
  hal::sensors::config::xml::ground_sensor_parser    m_ground{};
  /* clang-format on */
};

NS_END(xml, config, subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_CONFIG_XML_SENSING_SUBSYSTEM2D_PARSER_HPP_ */
