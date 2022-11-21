/**
 * \file positional_entropy_parser.hpp
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

#include "cosm/convergence/config/positional_entropy_config.hpp"
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::convergence::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class positional_entropy_parser
 * \ingroup convergence config xml
 *
 * \brief Parses XML configuration related the calculation of swarm positional
 * entropy into \ref positional_entropy_config.
 */
class positional_entropy_parser : public rer::client<positional_entropy_parser>,
                                  public rconfig::xml::xml_config_parser {
 public:
  using config_type = positional_entropy_config;

  positional_entropy_parser(void) : ER_CLIENT_INIT("cosm.convergence.config.xml.positional_entropy_parser") {}

  /**
   * \brief The root tag that all loop functions relating to positional_entropy
   * parameters should lie under in the XML tree.
   */
  static inline const std::string kXMLRoot = "positional_entropy";

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

} /* namespace cosm::convergence::config::xml */

