/**
 * \file entities_oracle_parser.hpp
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

#include "cosm/oracle/config/entities_oracle_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, oracle, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class entities_oracle_parser
 * \ingroup config oracle xml
 *
 * \brief Parses XML parameters used for the \ref entities_oracle at the start
 * of simulation.
 */
class entities_oracle_parser : public rer::client<entities_oracle_parser>,
                               public rconfig::xml::xml_config_parser {
 public:
  using config_type = entities_oracle_config;

  entities_oracle_parser(void) : ER_CLIENT_INIT("cosm.oracle.config.xml.entities_oracle_parser") {}

  /**
   * \brief The root tag that all entity oracle parameters should lie under in
   * the XML tree.
   */
  static inline const std::string kXMLRoot = "entities_oracle";

  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }
  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(xml, config, oracle, cosm);

