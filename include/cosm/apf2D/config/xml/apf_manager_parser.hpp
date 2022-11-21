/**
 * \file apf_manager_parser.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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
#include "cosm/apf2D/config/apf_manager_config.hpp"
#include "cosm/apf2D/nav/config/xml/nav_parser.hpp"
#include "cosm/apf2D/flocking/config/xml/flocking_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class apf_manager_parser
 * \ingroup apf2D config xml
 *
 * \brief Parses XML configuration for \ref apf_manager into
 * \ref apf_manager_config. Assumes it is handed an XML parent in which the
 * child tag \ref kXMLRoot is found.
 */
class apf_manager_parser : public rer::client<apf_manager_parser>,
                                public rconfig::xml::xml_config_parser {
 public:
  using config_type = apf_manager_config;

  apf_manager_parser(void)
      : ER_CLIENT_INIT("cosm.apf2D.config.xml.apf_manager_parser") {}

  /**
   * \brief The XML root tag that all \ref apf_manager configuration should
   * lie under in the XML tree.
   */
  static inline const std::string kXMLRoot = "apf_manager";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(cold, pure);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>           m_config{nullptr};
  nav::config::xml::nav_parser           m_nav{};
  flocking::config::xml::flocking_parser m_flocking{};
  /* clang-format on */
};

} /* namespace cosm::apf2D::config::xml */
