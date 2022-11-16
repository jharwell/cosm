/**
 * \file diff_drive_parser.hpp
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

#include "cosm/kin2D/config/diff_drive_config.hpp"
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, kin2D, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class diff_drive_parser
 * \ingroup kin2D config xml
 *
 * \brief Parses XML configuration relating to the \ref diff_drive into
 * \ref diff_drive_config.
 */
class diff_drive_parser : public rer::client<diff_drive_parser>,
                          public rconfig::xml::xml_config_parser {
 public:
  using config_type = diff_drive_config;

  diff_drive_parser(void) : ER_CLIENT_INIT("cosm.kin2D.config.xml.diff_drive_parser") {}

  /**
   * \brief The root tag that all diff drive configuration values
   * should lie under in the XML tree.
   */
  static inline const std::string kXMLRoot = "diff_drive";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(cold, pure);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(xml, config, kin2D, cosm);

