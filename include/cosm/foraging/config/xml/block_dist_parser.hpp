/**
 * \file block_dist_parser.hpp
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

#include "cosm/foraging/config/block_dist_config.hpp"
#include "cosm/foraging/config/xml/powerlaw_dist_parser.hpp"
#include "cosm/foraging/config/xml/block_manifest_parser.hpp"
#include "cosm/foraging/config/xml/block_redist_governor_parser.hpp"

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
 * \class block_dist_parser
 * \ingroup foraging config xml
 *
 * \brief Parses XML parameters related to block distribution
 * into \ref block_dist_config.
 */
class block_dist_parser : public rer::client<block_dist_parser>,
                          public rconfig::xml::xml_config_parser {
 public:
  using config_type = block_dist_config;

  block_dist_parser(void) : ER_CLIENT_INIT("cosm.foraging.config.xml.block_dist_parser") {}

  /**
   * \brief The root tag that all block distribution parameters should lie under
   * in the XML tree.
   */
  static inline const std::string kXMLRoot = "distribution";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  block_manifest_parser        m_manifest{};
  powerlaw_dist_parser         m_powerlaw{};
  block_redist_governor_parser m_redist_governor{};
  /* clang-format on */
};

} /* namespace cosm::foraging::config::xml */

