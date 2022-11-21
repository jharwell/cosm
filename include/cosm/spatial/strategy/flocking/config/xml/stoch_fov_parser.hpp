/**
 * \file stoch_fov_parser.hpp
 *
 * \copyright 2022 SIFT LLC, All rights reserved.
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

#include "cosm/spatial/strategy/flocking/config/stoch_fov_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::strategy::flocking::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class stoch_fov_parser
 * \ingroup spatial strategy flocking config xml
 *
 * \brief Parses XML configuration for \ref cssflocking::stoch_fov into
 * \ref flocking_config.
 */
class stoch_fov_parser final : public rer::client<stoch_fov_parser>,
                                    public rconfig::xml::xml_config_parser {
 public:
  using config_type = stoch_fov_config;

  stoch_fov_parser(void)
      : ER_CLIENT_INIT("cosm.spatial.strategy.flocking.config.xml.stoch_fov_parser") {}

  /**
   * \brief The root tag that all XML configuration for stoch FOV flocking
   * should lie under in the XML tree.
   */
  static inline const std::string kXMLRoot = "stoch_fov";

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

} /* namespace cosm::spatial::strategy::flocking::config::xml */
