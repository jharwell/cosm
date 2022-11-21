/**
 * \file drop_parser.hpp
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

#include "cosm/spatial/strategy/blocks/config/drop_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::strategy::blocks::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class drop_parser
 * \ingroup spatial strategy blocks config xml
 *
 * \brief Parses XML configuration for how robots should drop blocks into \ref
 * drop_config.
 */
class drop_parser final : public rer::client<drop_parser>,
                              public rconfig::xml::xml_config_parser {
 public:
  using config_type = drop_config;

  drop_parser(void)
      : ER_CLIENT_INIT("cosm.spatial.strategy.blocks.config.xml.drop_parser") {}

  /**
   * \brief The root tag that all XML configuration for nest acq should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "drop";

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

} /* namespace cosm::spatial::strategy::config::xml, blocks */
