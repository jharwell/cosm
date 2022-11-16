/**
 * \file force_calculator_parser.hpp
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
#include "cosm/steer2D/config/xml/force_calculator_parser.hpp"
#include "cosm/steer2D/config/force_calculator_config.hpp"
#include "cosm/steer2D/config/xml/avoidance_force_parser.hpp"
#include "cosm/steer2D/config/xml/arrival_force_parser.hpp"
#include "cosm/steer2D/config/xml/wander_force_parser.hpp"
#include "cosm/steer2D/config/xml/polar_force_parser.hpp"
#include "cosm/steer2D/config/xml/phototaxis_force_parser.hpp"
#include "cosm/steer2D/config/xml/path_following_force_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class force_calculator_parser
 * \ingroup steer2D config xml
 *
 * \brief Parses XML configuration for \ref force_calculator into
 * \ref force_calculator_config. Assumes it is handed an XML parent in which the
 * child tag \ref kXMLRoot is found.
 */
class force_calculator_parser : public rer::client<force_calculator_parser>,
                                public rconfig::xml::xml_config_parser {
 public:
  using config_type = force_calculator_config;

  force_calculator_parser(void) : ER_CLIENT_INIT("cosm.steer2D.config.xml.force_calculator_parser") {}

  /**
   * \brief The XML root tag that all \ref force_calculator configuration should
   * lie under in the XML tree.
   */
  static inline const std::string kXMLRoot = "force_calculator";

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

NS_END(xml, config, steer2D, cosm);

