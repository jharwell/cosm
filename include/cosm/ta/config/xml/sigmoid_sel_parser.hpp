/**
 * \file sigmoid_sel_parser.hpp
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
#include "rcppsw/math/config/xml/sigmoid_parser.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"
#include "cosm/ta/config/sigmoid_sel_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class sigmoid_sel_parser
 * \ingroup ta config xml
 *
 * \brief Parses XML configuration relating to probabilities built on
 * math::sigmoid to select between things into \ref sigmoid_sel_config.
 */
class sigmoid_sel_parser : public rer::client<sigmoid_sel_parser>,
                           public rcppsw::config::xml::xml_config_parser {
 public:
  using config_type = sigmoid_sel_config;

  sigmoid_sel_parser(void) : ER_CLIENT_INIT("cosm.ta.config.xml.sigmoid_sel_parser") {}

  /**
   * \brief The root tag that all XML configuration should lie under in the XML
   * tree.
   */
  static inline const std::string kXMLRoot = "sigmoid_sel";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);
  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD rcppsw::config::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>       m_config{nullptr};
  rmath::config::xml::sigmoid_parser m_sigmoid{};
  /* clang-format on */
};

NS_END(xml, config, ta, cosm);

