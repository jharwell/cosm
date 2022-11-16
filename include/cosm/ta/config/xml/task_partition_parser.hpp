/**
 * \file task_partition_parser.hpp
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
#include "cosm/ta/config/xml/src_sigmoid_sel_parser.hpp"
#include "cosm/ta/config/task_partition_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class task_partition_parser
 * \ingroup ta config xml
 *
 * \brief Parses XML configuration for relating to task partitioning into \ref
 * task_partition_config.
 */
class task_partition_parser final : public rer::client<task_partition_parser>,
                                    public rconfig::xml::xml_config_parser {
 public:
  using config_type = task_partition_config;

  task_partition_parser(void) : ER_CLIENT_INIT("cosm.ta.config.xml.task_partition_parser") {}

  /**
   * \brief The root tag that all task task_partition parameters should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "task_partition";

  bool validate(void) const override RCPPSW_COLD;
  void parse(const ticpp::Element& node) override RCPPSW_COLD;

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  src_sigmoid_sel_parser       m_sigmoid{};
  /* clang-format on */
};

NS_END(xml, config, ta, cosm);

