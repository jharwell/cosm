/**
 * \file tasking_oracle_parser.hpp
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

#include "cosm/oracle/config/tasking_oracle_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::oracle::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class tasking_oracle_parser
 * \ingroup oracle config xml
 *
 * \brief Parses XML parameters used for \ref tasking_oracle at the start of
 * simulation.
 */
class tasking_oracle_parser : public rer::client<tasking_oracle_parser>,
                              public rconfig::xml::xml_config_parser {
 public:
  using config_type = tasking_oracle_config;

  tasking_oracle_parser(void) : ER_CLIENT_INIT("cosm.oracle.config.xml.tasking_oracle_parser") {}

  /**
   * \brief The root tag that all cache parameters should lie under in the
   * XML tree.
   */
  static inline const std::string kXMLRoot = "tasking_oracle";

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

} /* namespace cosm::oracle::config::xml */

