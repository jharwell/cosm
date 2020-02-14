/**
 * \file actuation_subsystem2D_parser.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_SUBSYSTEM_CONFIG_ACTUATION_SUBSYSTEM2D_PARSER_HPP_
#define INCLUDE_COSM_SUBSYSTEM_CONFIG_ACTUATION_SUBSYSTEM2D_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "cosm/subsystem/config/actuation_subsystem2D_config.hpp"
#include "cosm/kin2D/config/xml/diff_drive_parser.hpp"
#include "cosm/steer2D/config/xml/force_calculator_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class actuation_subsystem2D_parser
 * \ingroup subsystem config xml
 *
 * \brief Parses XML parameters for \ref actuation_subsystem2D into
 * \ref actuation_subsystem2D_config.
 */
class actuation_subsystem2D_parser final : public rconfig::xml::xml_config_parser {
 public:
  using config_type = actuation_subsystem2D_config;

  /**
   * \brief The root tag that all 2D actuation subsystem parameters should lie
   * under in the XML tree.
   */
  static constexpr char kXMLRoot[] = "actuation_subsystem2D";

  bool validate(void) const override RCSW_PURE;
  void parse(const ticpp::Element& node) override RCSW_COLD;

  RCSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>                  m_config{nullptr};
  kin2D::config::xml::diff_drive_parser         m_diff_drive{};
  steer2D::config::xml::force_calculator_parser m_steering{};
  /* clang-format on */
};

NS_END(xml, config, subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_CONFIG_ACTUATION_SUBSYSTEM2D_PARSER_HPP_ */
