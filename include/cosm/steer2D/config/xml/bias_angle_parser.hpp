/**
 * \file bias_angle_parser.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foucndation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_COSM_STEER2D_CONFIG_XML_BIAS_ANGLE_PARSER_HPP_
#define INCLUDE_COSM_STEER2D_CONFIG_XML_BIAS_ANGLE_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "rcppsw/rcppsw.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"
#include "cosm/steer2D/config/bias_angle_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class bias_angle_parser
 * \ingroup steer2D config xml
 *
 * \brief Parses XML configuration for \ref bias_angle into
 * \ref bias_angle_config. Assumes it is handed an XML parent in which its
 * XML root tag is found.
 */
class bias_angle_parser final : public rer::client<bias_angle_parser>,
                                public rconfig::xml::xml_config_parser {
 public:
  using config_type = bias_angle_config;

  bias_angle_parser(void) : ER_CLIENT_INIT("cosm.steer2D.config.xml.bias_angle_parser") {}

  /**
   * \brief The XML root tag that all \ref bias_angle configuration should
   * lie under in the XML tree.
   */
  static inline const std::string kXMLRoot = "bias_angle";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;

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

#endif /* INCLUDE_COSM_STEER2D_CONFIG_XML_BIAS_ANGLE_PARSER_HPP_ */
