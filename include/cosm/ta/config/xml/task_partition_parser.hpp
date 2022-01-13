/**
 * \file task_partition_parser.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_COSM_TA_CONFIG_XML_TASK_PARTITION_PARSER_HPP_
#define INCLUDE_COSM_TA_CONFIG_XML_TASK_PARTITION_PARSER_HPP_

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

#endif /* INCLUDE_COSM_TA_CONFIG_XML_TASK_PARTITION_PARSER_HPP_ */
