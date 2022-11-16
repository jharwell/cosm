/**
 * \file bias_angle_parser.hpp
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
#include "cosm/apf2D/nav/config/bias_angle_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class bias_angle_parser
 * \ingroup apf2D nav config xml
 *
 * \brief Parses XML configuration for \ref bias_angle into
 * \ref bias_angle_config. Assumes it is handed an XML parent in which its
 * XML root tag is found.
 */
class bias_angle_parser final : public rer::client<bias_angle_parser>,
                                public rconfig::xml::xml_config_parser {
 public:
  using config_type = bias_angle_config;

  bias_angle_parser(void)
      : ER_CLIENT_INIT("cosm.apf2D.nav.config.xml.bias_angle_parser") {}

  /**
   * \brief The XML root tag that all \ref bias_angle configuration should
   * lie under in the XML tree.
   */
  static inline const std::string kXMLRoot = "bias_angle";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;

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
