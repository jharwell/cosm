/**
 * \file phototaxis_force_parser.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "cosm/steer2D/config/phototaxis_force_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, steer2D, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class phototaxis_force_parser
 * \ingroup steer2D config xml
 *
 * \brief Parses XML parameters for related to \ref phototaxis_force objects
 * into \ref phototaxis_force_config.
 */
class phototaxis_force_parser final : public rer::client<phototaxis_force_parser>,
                                      public rconfig::xml::xml_config_parser {
 public:
  using config_type = phototaxis_force_config;

  phototaxis_force_parser(void) : ER_CLIENT_INIT("cosm.steer2D.config.xml.phototaxis_force_parser") {}

  /**
   * \brief The root tag that all phototaxis_force parameters should lie under
   * in the XML tree.
   */
  static inline const std::string kXMLRoot = "phototaxis_force";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(cold, pure);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format on */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format off */
};

NS_END(xml, config, steer2D, cosm);

