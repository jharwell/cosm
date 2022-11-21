/**
 * \file exec_estimates_parser.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>
#include <utility>
#include <memory>


#include "rcppsw/math/config/xml/ema_parser.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"
#include "cosm/ta/config/exec_estimates_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class exec_estimates_parser
 * \ingroup ta config xml
 *
 * \brief Parses XML configuration used for estimation of task execution
 * times at the start of simulation.
 */
class exec_estimates_parser final : public rconfig::xml::xml_config_parser,
                                    public rer::client<exec_estimates_parser> {
 public:
  using config_type = exec_estimates_config;

  explicit exec_estimates_parser(
      std::list<std::string> task_names = std::list<std::string>()) noexcept
      : ER_CLIENT_INIT("cosm.ta.config.xml.exec_estimates_parser"),
        m_task_names(std::move(task_names)) {}

  /**
   * \brief The root tag that all cache parameters should lie under in the
   * XML tree.
   */
  static inline const std::string kXMLRoot = "task_exec_estimates";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

  RCPPSW_COLD void task_add(const std::string& task) {
    m_task_names.push_back(task);
  }

 private:
  RCPPSW_COLD rcppsw::config::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>   m_config{nullptr};
  rmath::config::xml::ema_parser m_ema{};
  std::list<std::string>         m_task_names;
  /* clang-format on */
};

} /* namespace cosm::ta::config::xml */

