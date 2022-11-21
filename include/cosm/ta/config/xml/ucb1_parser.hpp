/**
 * \file ucb1_parser.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "rcppsw/math/config/xml/sigmoid_parser.hpp"
#include "cosm/ta/config/ucb1_config.hpp"
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ucb1_parser
 * \ingroup ta config xml
 *
 * \brief Parses XML configuration used for epsilon greedy task allocation
 * policy into \ref ucb1_config.
 */
class ucb1_parser final : public rer::client<ucb1_parser>,
                          public rconfig::xml::xml_config_parser {
 public:
  using config_type = ucb1_config;

  ucb1_parser(void) : ER_CLIENT_INIT("cosm.ta.config.xml.ucb1_parser") {}

  /**
   * \brief The root tag that all task allocation XML configuration should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "ucb1";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

} /* namespace cosm::ta::config::xml */

