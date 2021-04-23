/**
 * \file metrics_parser.hpp
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

#ifndef INCLUDE_COSM_METRICS_CONFIG_XML_METRICS_PARSER_HPP_
#define INCLUDE_COSM_METRICS_CONFIG_XML_METRICS_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <memory>
#include <string>

#include "rcppsw/config/xml/xml_config_parser.hpp"

#include "cosm/metrics/config/metrics_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, metrics, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class metrics_parser
 * \ingroup cosm metrics config xml
 *
 * \brief Parses XML parameters related to metric collection into
 * \ref metrics_config.
 */
class metrics_parser : public rconfig::xml::xml_config_parser {
 public:
  using config_type = metrics_config;

  ~metrics_parser(void) override = default;

  /**
   * \brief The root tag that all loop functions relating to metrics parameters
   * should lie under in the XML tree.
   */
  inline static const std::string kXMLRoot = "metrics";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /**
   * \brief Determine if a particular attribute under the a node is the name of
   * a metric collector or some other type of parameter.
   */
  bool is_collector_name(const ticpp::Attribute& attr) const RCPPSW_COLD;

  void output_mode_parse(const ticpp::Element& element,
                         metrics_output_mode_config* config);

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(xml, config, metrics, cosm);

#endif /* INCLUDE_COSM_METRICS_CONFIG_XML_METRICS_PARSER_HPP_ */
