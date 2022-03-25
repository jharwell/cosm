/**
 * \file output_parser.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>
#include <list>

#include "rcppsw/config/xml/xml_config_parser.hpp"
#include "rcppsw/metrics/config/xml/metrics_parser.hpp"

#include "cosm/pal/config/output_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, pal, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class output_parser
 * \ingroup pal config xml
 *
 * \brief Parses XML parameters relating to simulation outputs into
 * \ref output_config.
 */
class output_parser final : public rer::client<output_parser>,
                            public rconfig::xml::xml_config_parser {
 public:
  using config_type = output_config;

  output_parser(void) : ER_CLIENT_INIT("cosm.pal.config.xml.output_parser") {}

  /**
   * \brief The root tag that all output loop functions parameters should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "output";

  bool validate(void) const override RCPPSW_ATTR(pure, cold);
  void parse(const ticpp::Element& node) override RCPPSW_COLD;

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<output_config> m_config{nullptr};
  rmcxml::metrics_parser         m_metrics_parser{};
  /* clang-format on */
};

NS_END(xml, config, pal, cosm);

