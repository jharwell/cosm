/**
 * \file pheromone_parser.hpp
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

#include "cosm/subsystem/perception/config/pheromone_config.hpp"
#include "cosm/cosm.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem, perception, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class pheromone_parser
 * \ingroup subsystem perception config xml
 *
 * \brief Parses XML parameters relating to pheromones into
 * \ref pheromone_config.
 */
class pheromone_parser : public rer::client<pheromone_parser>,
                         public rconfig::xml::xml_config_parser {
 public:
  using config_type = pheromone_config;

  pheromone_parser(void) : ER_CLIENT_INIT("cosm.subsystem.perception.config.xml.pheromone_parser") {}

  /**
   * \brief The root tag that all pheromone parameters should lie under in the
   * XML tree.
   */
  static inline const std::string kXMLRoot = "pheromone";

  bool validate(void) const override RCPPSW_ATTR(pure, cold);
  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

 private:
  /* clang-format off */
  std::shared_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(xml, config, perception, subsystem, cosm);

