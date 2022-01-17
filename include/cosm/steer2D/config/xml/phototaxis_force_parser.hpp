/**
 * \file phototaxis_force_parser.hpp
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

#ifndef INCLUDE_COSM_STEER2D_CONFIG_XML_PHOTOTAXIS_FORCE_PARSER_HPP_
#define INCLUDE_COSM_STEER2D_CONFIG_XML_PHOTOTAXIS_FORCE_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "cosm/steer2D/config/phototaxis_force_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, steer2D, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class phototaxis_force_parser
 * \ingroup steer2D config xml
 *
 * \brief Parses XML parameters for related to \ref phototaxis_force objects
 * into \ref phototaxis_force_config.
 */
class phototaxis_force_parser final : public rer::client<phototaxis_force_parser>,
                                      public rconfig::xml::xml_config_parser {
 public:
  using config_type = phototaxis_force_config;

  phototaxis_force_parser(void) : ER_CLIENT_INIT("cosm.steer2D.config.xml.phototaxis_force_parser") {}

  /**
   * \brief The root tag that all phototaxis_force parameters should lie under
   * in the XML tree.
   */
  static inline const std::string kXMLRoot = "phototaxis_force";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(cold, pure);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format on */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format off */
};

NS_END(xml, config, steer2D, cosm);

#endif /* INCLUDE_COSM_STEER2D_CONFIG_XML_PHOTOTAXIS_FORCE_PARSER_HPP_ */
