/**
 * \file temporal_penalty_parser.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
namespace cosm::tv::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class temporal_penalty_parser
 * \ingroup tv config xml
 *
 * \brief Parses XML configuration for \ref temporal_penalty into \ref
 * temporal_penalty_config.
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

} /* namespace cosm::tv::config::xml */

