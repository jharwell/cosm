/**
 * \file constant_speed_force_parser.hpp
 *
 * \copyright 2022 SIFT LLC, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "cosm/apf2D/flocking/config/constant_speed_force_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::apf2D::flocking::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class constant_speed_force_parser
 * \ingroup apf2D flocking config xml
 *
 * \brief Parses XML parameters for related to \ref constant_speed_force objects
 * into \ref constant_speed_force_config.
 */
class constant_speed_force_parser final : public rer::client<constant_speed_force_parser>,
                                          public rconfig::xml::xml_config_parser {
 public:
  using config_type = constant_speed_force_config;

  constant_speed_force_parser(void)
      : ER_CLIENT_INIT("cosm.apf2D.flocking.config.xml.constant_speed_force_parser") {}

  /**
   * \brief The root tag that all constant_speed_force parameters should lie under
   * in the XML tree.
   */
  static inline const std::string kXMLRoot = "constant_speed_force";

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

} /* namespace cosm::apf2D::flocking::config::xml */
