/**
 * \file explore_parser.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#include "cosm/spatial/strategy/explore/config/explore_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::strategy::explore::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class explore_parser
 * \ingroup spatial strategy explore config xml
 *
 * \brief Parses XML configuration exploration strategies into \ref
 * explore_config.
 */
class explore_parser final : public rer::client<explore_parser>,
                             public rconfig::xml::xml_config_parser {
 public:
  using config_type = explore_config;

  explore_parser(void)
      : ER_CLIENT_INIT("cosm.spatial.strategy.config.xml.explore_parser") {}

  /**
   * \brief The root tag that all XML configuration for nest acq should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "explore";

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

} /* namespace cosm::spatial::strategy::explore ::xml, config */
