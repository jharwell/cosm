/**
 * @file angular_order_parser.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_CONVERGENCE_CONFIG_XML_ANGULAR_ORDER_PARSER_HPP_
#define INCLUDE_COSM_CONVERGENCE_CONFIG_XML_ANGULAR_ORDER_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "cosm/convergence/config/angular_order_config.hpp"
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, convergence, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class angular_order_parser
 * @ingroup cosm convergence config xml
 *
 * @brief Parses XML configuration related to the calculation of swarm angular
 * order into \ref angular_order_config.
 */
class angular_order_parser : public rconfig::xml::xml_config_parser {
 public:
  using config_type = angular_order_config;

  /**
   * @brief The root tag that all XML configuration for angular order objects
   * should lie under in the XML tree.
   */
  static constexpr char kXMLRoot[] = "angular_order";

  void parse(const ticpp::Element& node) override RCSW_COLD;

  RCSW_COLD std::string xml_root(void) const override { return kXMLRoot; }


 private:
  RCSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(xml, config, convergence, cosm);

#endif /* INCLUDE_COSM_CONVERGENCE_CONFIG_XML_ANGULAR_ORDER_PARSER_HPP_ */
