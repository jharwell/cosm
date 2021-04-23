/**
 * \file force_calculator_parser.hpp
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

#ifndef INCLUDE_COSM_STEER2D_CONFIG_XML_FORCE_CALCULATOR_PARSER_HPP_
#define INCLUDE_COSM_STEER2D_CONFIG_XML_FORCE_CALCULATOR_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "rcppsw/rcppsw.hpp"
#include "cosm/steer2D/config/xml/force_calculator_parser.hpp"
#include "cosm/steer2D/config/force_calculator_config.hpp"
#include "cosm/steer2D/config/xml/avoidance_force_parser.hpp"
#include "cosm/steer2D/config/xml/arrival_force_parser.hpp"
#include "cosm/steer2D/config/xml/wander_force_parser.hpp"
#include "cosm/steer2D/config/xml/polar_force_parser.hpp"
#include "cosm/steer2D/config/xml/phototaxis_force_parser.hpp"
#include "cosm/steer2D/config/xml/path_following_force_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class force_calculator_parser
 * \ingroup steer2D config xml
 *
 * \brief Parses XML configuration for \ref force_calculator into
 * \ref force_calculator_config. Assumes it is handed an XML parent in which the
 * child tag \ref kXMLRoot is found.
 */
class force_calculator_parser : public rconfig::xml::xml_config_parser {
 public:
  using config_type = force_calculator_config;

  /**
   * \brief The XML root tag that all \ref force_calculator configuration should
   * lie under in the XML tree.
   */
  inline static const std::string kXMLRoot = "force_calculator";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(cold, pure);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  avoidance_force_parser       m_avoidance{};
  arrival_force_parser         m_arrival{};
  wander_force_parser          m_wander{};
  polar_force_parser           m_polar{};
  phototaxis_force_parser      m_phototaxis{};
  path_following_force_parser  m_path_following{};
  /* clang-format on */
};

NS_END(xml, config, steer2D, cosm);

#endif /* INCLUDE_COSM_STEER2D_CONFIG_XML_FORCE_CALCULATOR_PARSER_HPP_ */
