/**
 * \file stoch_nbhd1_parser.hpp
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
#include "rcppsw/config/xml/xml_config_parser.hpp"
#include "cosm/ta/config/xml/src_sigmoid_sel_parser.hpp"
#include "cosm/ta/config/stoch_nbhd1_config.hpp"
#include "cosm/ta/config/xml/task_partition_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class stoch_nbhd1_parser
 * \ingroup ta config xml
 *
 * \brief Parses XML configuration used for task allocation into \ref
 * stoch_nbhd1_config.
 */
class stoch_nbhd1_parser final : public rer::client<stoch_nbhd1_parser>,
                                 public rconfig::xml::xml_config_parser {
 public:
  using config_type = stoch_nbhd1_config;

  stoch_nbhd1_parser(void) : ER_CLIENT_INIT("cosm.ta.config.xml.stoch_nbhd1_parser") {}

  /**
   * \brief The root tag that all task allocation XML configuration should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "stoch_nbhd1";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD rcppsw::config::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  src_sigmoid_sel_parser       m_subtask_sel{};
  task_partition_parser        m_partitioning{};
  src_sigmoid_sel_parser       m_tab_sel{};
  /* clang-format on */
};

} /* namespace cosm::ta::config::xml */

