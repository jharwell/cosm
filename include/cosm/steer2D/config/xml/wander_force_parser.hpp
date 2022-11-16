/**
 * \file wander_force_parser.hpp
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
#include "cosm/steer2D/config/wander_force_config.hpp"
#include "cosm/steer2D/config/xml/bias_angle_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class wander_force_parser
 * \ingroup steer2D config xml
 *
 * \brief Parses XML configuration for \ref wander_force into
 * \ref wander_force_config. Assumes it is handed an XML parent in which the
 * child tag \ref kXMLRoot is found.
 */
class wander_force_parser final : public rer::client<wander_force_parser>,
                                  public rconfig::xml::xml_config_parser {
 public:
  using config_type = wander_force_config;

  wander_force_parser(void) : ER_CLIENT_INIT("cosm.steer2D.config.xml.wander_force_parser") {}

  /**
   * \brief The XML root tag that all \ref wander_force configuration should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "wander_force";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(cold, pure);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  bias_angle_parser            m_bias{};
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(xml, config, steer2D, cosm);

