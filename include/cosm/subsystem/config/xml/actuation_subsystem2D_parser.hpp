/**
 * \file actuation_subsystem2D_parser.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "cosm/subsystem/config/actuation_subsystem2D_config.hpp"
#include "cosm/kin2D/config/xml/diff_drive_parser.hpp"
#include "cosm/steer2D/config/xml/force_calculator_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class actuation_subsystem2D_parser
 * \ingroup subsystem config xml
 *
 * \brief Parses XML parameters for \ref actuation_subsystem2D into
 * \ref actuation_subsystem2D_config.
 */
class actuation_subsystem2D_parser final : public rer::client<actuation_subsystem2D_parser>,
                                           public rconfig::xml::xml_config_parser {
 public:
  using config_type = actuation_subsystem2D_config;

  actuation_subsystem2D_parser(void) : ER_CLIENT_INIT("cosm.subsystem.config.xml.actuation_subsystem2D_parser") {}

  /**
   * \brief The root tag that all 2D actuation subsystem parameters should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "actuation_subsystem2D";

  bool validate(void) const override RCPPSW_ATTR(pure, cold);
  void parse(const ticpp::Element& node) override RCPPSW_COLD;

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>                  m_config{nullptr};
  kin2D::config::xml::diff_drive_parser         m_diff_drive{};
  steer2D::config::xml::force_calculator_parser m_steering{};
  /* clang-format on */
};

NS_END(xml, config, subsystem, cosm);

