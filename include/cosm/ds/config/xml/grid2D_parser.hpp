/**
 * \file grid2D_parser.hpp
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

#ifndef INCLUDE_COSM_DS_CONFIG_XML_GRID2D_PARSER_HPP_
#define INCLUDE_COSM_DS_CONFIG_XML_GRID2D_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "rcppsw/config/xml/xml_config_parser.hpp"

#include "cosm/ds/config/grid2D_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ds, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class grid2D_parser
 * \ingroup ds config xml
 *
 * \brief Parses XML parameters for \ref arena_grid grid structures into \ref
 * grid2D_config.
 */

class grid2D_parser : public rer::client<grid2D_parser>,
                      public rconfig::xml::xml_config_parser {
 public:
  using config_type = grid2D_config;

  grid2D_parser(void) : ER_CLIENT_INIT("cosm.ds.config.xml.grid2D_parser") {}

  /**
   * \brief The root tag that all grid parameters should lie under in the
   * XML tree.
   */
  static inline const std::string kXMLRoot = "grid2D";

  bool validate(void) const override RCPPSW_ATTR(pure, cold);
  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<grid2D_config> m_config{nullptr};
  /* clang-format on */
};

NS_END(xml, config, ds, cosm);

#endif /* INCLUDE_COSM_DS_CONFIG_XML_GRID2D_PARSER_HPP_ */
