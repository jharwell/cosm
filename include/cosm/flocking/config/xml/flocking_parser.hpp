/**
 * \file flocking_parser.hpp
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

#include "cosm/flocking/config/flocking_config.hpp"
#include "cosm/flocking/config/xml/stoch_fov_parser.hpp"
#include "cosm/nav/config/xml/trajectory_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::flocking::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class flocking_parser
 * \ingroup flocking config xml
 *
 * \brief Parses XML configuration for how robots should perform flocking into
 * \ref flocking_config.
 */
class flocking_parser final : public rer::client<flocking_parser>,
                              public rconfig::xml::xml_config_parser {
 public:
  using config_type = flocking_config;

  flocking_parser(void)
      : ER_CLIENT_INIT("cosm.flocking.config.xml.flocking_parser") {}

  /**
   * \brief The root tag that all XML configuration for flocking should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "flocking";

  void parse(const ticpp::Element& node) override;
  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>     m_config{nullptr};

  stoch_fov_parser                 m_stoch_fov{};
  cnconfig::xml::trajectory_parser m_trajectory{};
  /* clang-format on */
};

} /* namespace cosm::flocking::config::xml */
