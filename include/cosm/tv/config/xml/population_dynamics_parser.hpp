/**
 * \file population_dynamics_parser.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "cosm/tv/config/population_dynamics_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, tv, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class population_dynamics_parser
 * \ingroup tv config xml
 *
 * \brief Parses XML configuration for \ref population_dynamics into \ref
 * population_dynamics_config. Assumes it is handed an XML parent in which
 * the child tag \ref kXMLRoot is found.
 *
 * Parameter tags under the XML root are expected to exactly match the names of
 * the fields in the \ref population_dynamics_config struct.
 */
class population_dynamics_parser final : public rer::client<population_dynamics_parser>,
                                         public rconfig::xml::xml_config_parser {
 public:
  using config_type = population_dynamics_config;

  population_dynamics_parser(void) : ER_CLIENT_INIT("cosm.tv.config.xml.population_dynamics_parser") {}

  /**
   * \brief The XML root tag that all \ref population_dynamics
   * configuration should lie under in the XML tree.
   */
  static inline const std::string kXMLRoot = "population_dynamics";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(xml, config, tv, cosm);

