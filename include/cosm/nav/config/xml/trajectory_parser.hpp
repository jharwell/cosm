/**
 * \file trajectory_parser.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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

#include "cosm/nav/config/trajectory_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::nav::config::xml {


/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class trajectory_parser
 * \ingroup nav config xml
 *
 * \brief Parses XML configuration for agent motion trajectories into
 * \ref trajectory_config.
 */
class trajectory_parser final : public rer::client<trajectory_parser>,
                                public rconfig::xml::xml_config_parser {
 public:
  using config_type = trajectory_config;

  trajectory_parser(void)
      : ER_CLIENT_INIT("cosm.nav.config.xml.trajectory_parser") {}

  /**
   * \brief The root tag that all XML configuration for trajectory should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "trajectory";

  void parse(const ticpp::Element& node) override;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);
  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

} /* namespace cosm::nav::config::xml */
