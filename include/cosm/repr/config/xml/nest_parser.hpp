/**
 * \file nest_parser.hpp
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

#include "cosm/repr/config/nest_config.hpp"
#include "cosm/cosm.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class nest_parser
 * \ingroup repr config xml
 *
 * \brief Parses XML parameters for related to \ref nest objects into
 * \ref nest_config.
 */
class nest_parser final : public rer::client<nest_parser>,
                          public rconfig::xml::xml_config_parser {
 public:
  using config_type = nest_config;

  nest_parser(void) : ER_CLIENT_INIT("cosm.repr.config.xml.nest_parser") {}

  /**
   * \brief The root tag that all \ref crepr::nest parameters should lie under
   * in the XML tree.
   */
  static inline const std::string kXMLRoot = "nest";

  static bool validate(const nest_config* config) RCPPSW_ATTR(pure, cold);

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<nest_config> m_config{nullptr};
  /* clang-format on */
};

} /* namespace cosm::repr::config::xml */

