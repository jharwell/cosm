/**
 * \file powerlaw_dist_parser.hpp
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

#include "cosm/foraging/config/powerlaw_dist_config.hpp"
#include "cosm/cosm.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class powerlaw_dist_parser
 * \ingroup foraging config xml
 *
 * \brief Parses XML parameters for related to \ref cfbd::powerlaw_distributor
 * objects into \ref powerlaw_dist_config.
 */
class powerlaw_dist_parser final : public rer::client<powerlaw_dist_parser>,
                                   public rconfig::xml::xml_config_parser {
 public:
  using config_type = powerlaw_dist_config;

  powerlaw_dist_parser(void) : ER_CLIENT_INIT("cosm.foraging.config.xml.powerlaw_dist_parser") {}

  /**
   * \brief The root tag that all powerlaw dist parameters should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "powerlaw";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<powerlaw_dist_config> m_config{nullptr};
  /* clang-format on */
};

} /* namespace cosm::foraging::config::xml */

