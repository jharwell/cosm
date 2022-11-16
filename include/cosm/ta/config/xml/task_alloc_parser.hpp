/**
 * \file task_alloc_parser.hpp
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

#include "rcppsw/math/config/xml/sigmoid_parser.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"
#include "cosm/ta/config/xml/exec_estimates_parser.hpp"
#include "cosm/ta/config/xml/src_sigmoid_sel_parser.hpp"
#include "cosm/ta/config/task_alloc_config.hpp"
#include "cosm/ta/config/xml/stoch_nbhd1_parser.hpp"
#include "cosm/ta/config/xml/epsilon_greedy_parser.hpp"
#include "cosm/ta/config/xml/ucb1_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class task_alloc_parser
 * \ingroup ta config xml
 *
 * \brief Parses XML configuration used for task allocation into \ref
 * task_alloc_config.
 */
class task_alloc_parser final : public rer::client<task_alloc_parser>,
                                public rconfig::xml::xml_config_parser {
 public:
  using config_type = task_alloc_config;

  task_alloc_parser(void) : ER_CLIENT_INIT("cosm.ta.config.xml.task_alloc_parser") {}

  /**
   * \brief The root tag that all task allocation XML configuration should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "task_alloc";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

  RCPPSW_COLD void exec_est_task_add(const std::string& task) {
    m_estimation.task_add(task);
  }

 private:
  RCPPSW_COLD rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  exec_estimates_parser        m_estimation{};
  src_sigmoid_sel_parser       m_abort{};
  stoch_nbhd1_parser           m_snbhd1{};
  epsilon_greedy_parser        m_epsilon{};
  ucb1_parser                  m_ucb1{};
  /* clang-format on */
};

NS_END(xml, config, ta, cosm);

