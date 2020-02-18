/**
 * \file block_dist_parser.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_COSM_FORAGING_CONFIG_BLOCK_DIST_PARSER_HPP_
#define INCLUDE_COSM_FORAGING_CONFIG_BLOCK_DIST_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "cosm/foraging/config/block_dist_config.hpp"
#include "cosm/foraging/config/powerlaw_dist_parser.hpp"
#include "cosm/foraging/config/block_manifest_parser.hpp"
#include "cosm/foraging/config/block_redist_governor_parser.hpp"

#include "cosm/cosm.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_dist_parser
 * \ingroup foraging config
 *
 * \brief Parses XML parameters related to block distribution
 * into \ref block_dist_config.
 */
class block_dist_parser : public rconfig::xml::xml_config_parser {
 public:
  using config_type = block_dist_config;

  /**
   * \brief The root tag that all block distribution parameters should lie under
   * in the XML tree.
   */
  static constexpr char kXMLRoot[] = "distribution";

  void parse(const ticpp::Element& node) override RCSW_COLD;
  bool validate(void) const override RCSW_ATTR(pure, cold);

  RCSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  block_manifest_parser        m_manifest{};
  powerlaw_dist_parser         m_powerlaw{};
  block_redist_governor_parser m_redist_governor{};
  /* clang-format on */
};

NS_END(config, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_CONFIG_BLOCK_DIST_PARSER_HPP_ */
