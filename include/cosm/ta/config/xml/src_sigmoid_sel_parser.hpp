/**
 * \file src_sigmoid_sel_parser.hpp
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
#include "cosm/ta/config/xml/sigmoid_sel_parser.hpp"
#include "cosm/ta/config/src_sigmoid_sel_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class src_sigmoid_sel_parser
 * \ingroup ta config xml
 *
 * \brief Parses XML configuration relating to sourced sigmoid selection of
 * stuff into \ref src_sigmoid_sel_config.
 */
class src_sigmoid_sel_parser : public rer::client<src_sigmoid_sel_parser>,
                               public rcppsw::config::xml::xml_config_parser {
 public:
  using config_type = src_sigmoid_sel_config;

  src_sigmoid_sel_parser(void) : ER_CLIENT_INIT("cosm.ta.config.xml.src_sigmoid_sel_parser") {}

  /**
   * \brief The root tag that all XML configuration should lie under in the XML
   * tree.
   */
  static inline const std::string kXMLRoot = "src_sigmoid_sel";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);
  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD rcppsw::config::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  sigmoid_sel_parser           m_sigmoid{};
  /* clang-format on */
};

} /* namespace cosm::ta::config::xml */

