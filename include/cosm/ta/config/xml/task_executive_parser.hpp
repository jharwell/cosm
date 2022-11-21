/**
 * \file task_executive_parser.hpp
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

#include "rcppsw/common/common.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"
#include "cosm/ta/config/task_executive_config.hpp"
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class task_executive_parser
 * \ingroup ta config xml
 *
 * \brief Parses XML configuration for \ref base_executive and its derived
 * classes into \ref task_executive_config.
 */
class task_executive_parser final : public rer::client<task_executive_parser>,
                                    public rconfig::xml::xml_config_parser {
 public:
  using config_type = task_executive_config;

  task_executive_parser(void) : ER_CLIENT_INIT("cosm.ta.config.xml.task_executive_parser") {}

  /**
   * \brief The root tag that all task task_executive parameters should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "task_executive";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
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

