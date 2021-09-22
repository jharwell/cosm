/**
 * \file interactivity_parser.hpp
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

#ifndef INCLUDE_COSM_CONVERGENCE_CONFIG_XML_INTERACTIVITY_PARSER_HPP_
#define INCLUDE_COSM_CONVERGENCE_CONFIG_XML_INTERACTIVITY_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "cosm/convergence/config/interactivity_config.hpp"
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
 * \class interactivity_parser
 * \ingroup convergence config xml
 *
 * \brief Parses XML configuration related the calculation of swarm interactivty
 * into \ref interactivity_config.
 */
class interactivity_parser : public rconfig::xml::xml_config_parser {
 public:
  using config_type = interactivity_config;

  /**
   * \brief The root tag that all XML configuration relating to interactivity
   * parameters should lie under in the XML tree.
   */
  static inline const std::string kXMLRoot = "interactivity";

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

NS_END(xml, config, convergence, cosm);

#endif /* INCLUDE_COSM_CONVERGENCE_CONFIG_XML_INTERACTIVITY_PARSER_HPP_ */
