/**
 * \file polar_force_parser.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "rcppsw/rcppsw.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"
#include "cosm/steer2D/config/polar_force_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class polar_force_parser
 * \ingroup steer2D config xml
 *
 * \brief Parses XML configuration for \ref polar_force into \ref
 * polar_force_config. Assumes it is handed an XML parent in which the child tag
 * \ref kXMLRoot is found.
 */
class polar_force_parser final : public rer::client<polar_force_parser>,
                                 public rconfig::xml::xml_config_parser {
 public:
  using config_type = polar_force_config;

  polar_force_parser(void) : ER_CLIENT_INIT("cosm.steer2D.config.xml.polar_force_parser") {}

  /**
   * \brief The XML root tag that all \ref polar_force configuration should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "polar_force";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(cold, pure);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(xml, config, steer2D, cosm);

