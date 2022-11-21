/**
 * \file acq_parser.hpp
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

#include "cosm/spatial/strategy/nest/config/acq_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::strategy::nest::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class acq_parser
 * \ingroup spatial strategy nest config xml
 *
 * \brief Parses XML configuration for how robots should acq nests into \ref
 * acq_config.
 */
class acq_parser final : public rer::client<acq_parser>,
                              public rconfig::xml::xml_config_parser {
 public:
  using config_type = acq_config;

  acq_parser(void)
      : ER_CLIENT_INIT("cosm.spatial.strategy.nest.config.xml.acq_parser") {}

  /**
   * \brief The root tag that all XML configuration for nest acq should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "acq";

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

} /* namespace cosm::spatial::strategy::nest::xml, config */
