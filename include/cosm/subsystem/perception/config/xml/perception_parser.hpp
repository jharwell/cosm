/**
 * \file perception_parser.hpp
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

#ifndef INCLUDE_COSM_SUBSYSTEM_PERCEPTION_CONFIG_PERCEPTION_PARSER_HPP_
#define INCLUDE_COSM_SUBSYSTEM_PERCEPTION_CONFIG_PERCEPTION_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "rcppsw/config/xml/xml_config_parser.hpp"

#include "cosm/ds/config/xml/grid2D_parser.hpp"
#include "cosm/subsystem/perception/config/perception_config.hpp"
#include "cosm/subsystem/perception/config/xml/pheromone_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem, perception, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class perception_parser
 * \ingroup subsystem perception config xml
 *
 * \brief Parses XML parameters for various perception subsystems into
 * \ref perception_config.
 */
class perception_parser final : public rconfig::xml::xml_config_parser {
 public:
  using config_type = perception_config;

  /**
   * \brief The root tag that all perception  parameters should lie under in
   * the XML tree.
   */
  static constexpr char kXMLRoot[] = "perception";

  bool validate(void) const override RCSW_ATTR(pure, cold);
  void parse(const ticpp::Element& node) override RCSW_COLD;

  RCSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  cdconfig::xml::grid2D_parser m_grid{};
  pheromone_parser             m_pheromone{};
  /* clang-format on */
};

NS_END(xml, config, perception, subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_CONFIG_PERCEPTION_XML_PERCEPTION_PARSER_HPP_ */