/**
 * \file temporal_penalty_parser.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "rcppsw/control/config/xml/waveform_parser.hpp"

#include "cosm/tv/config/temporal_penalty_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, tv, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class temporal_penalty_parser
 * \ingroup tv config xml
 *
 * \brief Parses XML configuration for \ref temporal_penalty into \ref
 * temporal_penalty_config. Assumes it is handed an XML parent in which
 * the child tag \ref kXMLRoot is found.
 *
 * Parameter tags under the XML root are expected to exactly match the names of
 * the fields in the \ref temporal_penalty_config struct.
 */
class temporal_penalty_parser final : public rer::client<temporal_penalty_parser>,
                                      public rconfig::xml::xml_config_parser {
 public:
  using config_type = temporal_penalty_config;

  temporal_penalty_parser(void) : ER_CLIENT_INIT("cosm.tv.config.xml.temporal_penalty_parser") {}


  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);

  /**
   * \brief The XML root tag that all \ref temporal_penalty
   * configuration should lie under in the XML tree.
   */
  RCPPSW_COLD std::string xml_root(void) const override { return m_xml_root; }
  RCPPSW_COLD void xml_root(const std::string& xml_root) { m_xml_root = xml_root; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  rct::config::xml::waveform_parser m_waveform{};
  std::unique_ptr<config_type>      m_config{nullptr};
  std::string                       m_xml_root{};
  /* clang-format on */
};

NS_END(xml, config, tv, cosm);

