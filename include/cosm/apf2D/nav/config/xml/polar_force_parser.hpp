/**
 * \file polar_force_parser.hpp
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

#include "rcppsw/rcppsw.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"
#include "cosm/apf2D/nav/config/polar_force_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class polar_force_parser
 * \ingroup apf2D nav config xml
 *
 * \brief Parses XML configuration for \ref polar_force into \ref
 * polar_force_config. Assumes it is handed an XML parent in which the child tag
 * \ref kXMLRoot is found.
 */
class polar_force_parser final : public rer::client<polar_force_parser>,
                                 public rconfig::xml::xml_config_parser {
 public:
  using config_type = polar_force_config;

  polar_force_parser(void)
      : ER_CLIENT_INIT("cosm.apf2D.nav.config.xml.polar_force_parser") {}

  /**
   * \brief The XML root tag that all \ref polar_force configuration should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "polar_force";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(cold, pure);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

} /* namespace cosm::apf2D::nav::config::xml */
