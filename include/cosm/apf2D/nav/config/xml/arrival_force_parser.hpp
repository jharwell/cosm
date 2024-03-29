/**
 * \file arrival_force_parser.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "rcppsw/rcppsw.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"
#include "cosm/apf2D/nav/config/arrival_force_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class arrival_force_parser
 * \ingroup apf2D nav config xml
 *
 * \brief Parses XML configuration for \ref arrival_force into \ref
 * arrival_force_config. Assumes it is handed an XML parent in which the child
 * tag \ref kXMLRoot is found.
 *
 * Parameter tags under the XML root are expected to exactly match the names of
 * the fields in the \ref arrival_force_config struct.
 */
class arrival_force_parser final : public rer::client<arrival_force_parser>,
                                   public rconfig::xml::xml_config_parser {
 public:
  using config_type = arrival_force_config;

  arrival_force_parser(void)
      : ER_CLIENT_INIT("cosm.apf2D.nav.config.xml.arrival_force_parser") {}

  /**
   * \brief The XML root tag that all \ref arrival_force configuration should
   * lie under in the XML tree.
   */
  static inline const std::string kXMLRoot = "arrival_force";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

} /* namespace cosm::apf2D::nav::config::xml */
