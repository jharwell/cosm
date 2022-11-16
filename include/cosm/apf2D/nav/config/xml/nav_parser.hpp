/**
 * \file nav_parser.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "cosm/apf2D/nav/config/nav_config.hpp"
#include "cosm/apf2D/nav/config/xml/avoidance_force_parser.hpp"
#include "cosm/apf2D/nav/config/xml/arrival_force_parser.hpp"
#include "cosm/apf2D/nav/config/xml/wander_force_parser.hpp"
#include "cosm/apf2D/nav/config/xml/polar_force_parser.hpp"
#include "cosm/apf2D/nav/config/xml/phototaxis_force_parser.hpp"
#include "cosm/apf2D/nav/config/xml/path_following_force_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class nav_parser
 * \ingroup apf2D nav config xml
 *
 * \brief Parses XML configuration for \ref nav into
 * \ref nav_config. Assumes it is handed an XML parent in which the
 * child tag \ref kXMLRoot is found.
 */
class nav_parser : public rer::client<nav_parser>,
                   public rconfig::xml::xml_config_parser {
 public:
  using config_type = nav_config;

  nav_parser(void)
      : ER_CLIENT_INIT("cosm.apf2D.config.xml.nav_parser") {}

  /**
   * \brief The XML root tag that all \ref nav configuration should
   * lie under in the XML tree.
   */
  static inline const std::string kXMLRoot = "nav";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(cold, pure);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  avoidance_force_parser       m_avoidance{};
  arrival_force_parser         m_arrival{};
  wander_force_parser          m_wander{};
  polar_force_parser           m_polar{};
  phototaxis_force_parser      m_phototaxis{};
  path_following_force_parser  m_path_following{};
  /* clang-format on */
};

} /* namespace cosm::apf2D::nav::config::xml */
