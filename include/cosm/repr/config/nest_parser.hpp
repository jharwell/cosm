/**
 * \file nest_parser.hpp
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

#ifndef INCLUDE_COSM_REPR_CONFIG_NEST_PARSER_HPP_
#define INCLUDE_COSM_REPR_CONFIG_NEST_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "cosm/repr/config/nest_config.hpp"
#include "cosm/cosm.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class nest_parser
 * \ingroup repr config
 *
 * \brief Parses XML parameters for related to \ref nest objects into
 * \ref nest_config.
 */
class nest_parser : public rconfig::xml::xml_config_parser {
 public:
  using config_type = nest_config;

  /**
   * \brief The root tag that all nest parameters should lie under in the
   * XML tree.
   */
  static constexpr char kXMLRoot[] = "nest";

  void parse(const ticpp::Element& node) override RCSW_COLD;
  bool validate(void) const override RCSW_ATTR(pure, cold);

  RCSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<nest_config> m_config{nullptr};
  /* clang-format on */
};

NS_END(config, repr, cosm);

#endif /* INCLUDE_COSM_REPR_CONFIG_NEST_PARSER_HPP_ */
