/**
 * \file aggregate_oracle_parser.hpp
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

#include "cosm/oracle/config/aggregate_oracle_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"
#include "cosm/oracle/config/xml/entities_oracle_parser.hpp"
#include "cosm/oracle/config/xml/tasking_oracle_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::oracle::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class aggregate_oracle_parser
 * \ingroup oracle config xml
 *
 * \brief Parses XML parameters used for oracles at the start of simulation.
 */
class aggregate_oracle_parser final : public rer::client<aggregate_oracle_parser>,
                                      public rconfig::xml::xml_config_parser {
 public:
  using config_type = aggregate_oracle_config;

  aggregate_oracle_parser(void)
      : ER_CLIENT_INIT("cosm.oracle.config.xml.aggregate_oracle_parser") {}

  /**
   * \brief The root tag that all oracle parameters should lie under in the
   * XML tree.
   */
  static inline const std::string kXMLRoot = "aggregate_oracle";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(const, cold);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>          m_config{nullptr};
  coconfig::xml::entities_oracle_parser m_entities{};
  coconfig::xml::tasking_oracle_parser  m_tasking{};
  /* clang-format on */
};

} /* namespace cosm::oracle::config::xml */
