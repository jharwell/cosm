/**
 * \file epsilon_greedy_parser.hpp
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
#include "cosm/ta/config/epsilon_greedy_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class epsilon_greedy_parser
 * \ingroup ta config xml
 *
 * \brief Parses XML configuration used for epsilon greedy task allocation
 * policy into \ref epsilon_greedy_config.
 */
class epsilon_greedy_parser final : public rer::client<epsilon_greedy_parser>,
                                    public rcppsw::config::xml::xml_config_parser {
 public:
  using config_type = epsilon_greedy_config;

  epsilon_greedy_parser(void) : ER_CLIENT_INIT("cosm.ta.config.xml.epsilon_greedy_parser") {}

  /**
   * \brief The root tag that all task allocation XML configuration should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "epsilon_greedy";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD rcppsw::config::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

} /* namespace cosm::ta::config::xml */

