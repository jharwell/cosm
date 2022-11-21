/**
 * \file flocking_parser.hpp
 *
 * \copyright 2022 SIFT LLC, All rights reserved.
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
#include "cosm/apf2D/flocking/config/flocking_config.hpp"
#include "cosm/apf2D/flocking/config/xml/alignment_force_parser.hpp"
#include "cosm/apf2D/flocking/config/xml/constant_speed_force_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::flocking::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class flocking_parser
 * \ingroup apf2D flocking config xml
 *
 * \brief Parses XML configuration for flocking forces into
 * \ref flocking_config. Assumes it is handed an XML parent in which the
 * child tag \ref kXMLRoot is found.
 */
class flocking_parser : public rer::client<flocking_parser>,
                   public rconfig::xml::xml_config_parser {
 public:
  using config_type = flocking_config;

  flocking_parser(void)
      : ER_CLIENT_INIT("cosm.apf2D.config.xml.flocking_parser") {}

  /**
   * \brief The XML root tag that all \ref flocking configuration should
   * lie under in the XML tree.
   */
  static inline const std::string kXMLRoot = "flocking";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(cold, pure);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  alignment_force_parser       m_alignment{};
  constant_speed_force_parser  m_constant_speed{};
  /* clang-format on */
};

} /* namespace cosm::apf2D::flocking::config::xml */
