/**
 * \file visualization_parser.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "rcppsw/config/xml/xml_config_parser.hpp"

#include "cosm/argos/vis/config/visualization_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, argos, vis, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class visualization_parser
 * \ingroup vis config xml
 *
 * \brief Parses XML parameters relating to visualization in loop functions into
 * \ref visualization_config.
 */
class visualization_parser final : public rer::client<visualization_parser>,
                                   public rconfig::xml::xml_config_parser {
 public:
  using config_type = visualization_config;

  visualization_parser(void) : ER_CLIENT_INIT("cosm.vis.config.xml.visualization_parser") {}

  /**
   * \brief The root tag that all visualization loop functions parameters should
   * lie under in the XML tree.
   */
  static inline const std::string kXMLRoot = "visualization";

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

NS_END(xml, config, vis, argos, cosm);

