/**
 * \file velocity_parser.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "cosm/convergence/config/velocity_config.hpp"
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, convergence, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class velocity_parser
 * \ingroup convergence config xml
 *
 * \brief Parses XML configuration related to the calculation of swarm angular
 * order into \ref velocity_config.
 */
class velocity_parser : public rer::client<velocity_parser>,
                        public rconfig::xml::xml_config_parser {
 public:
  using config_type = velocity_config;

  velocity_parser(void) : ER_CLIENT_INIT("cosm.convergence.config.xml.velocity_parser") {}

  /**
   * \brief The root tag that all XML configuration should lie under in the XML
   * tree.
   */
  static inline const std::string kXMLRoot = "velocity";

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

NS_END(xml, config, convergence, cosm);

