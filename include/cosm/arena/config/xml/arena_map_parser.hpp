/**
 * \file arena_map_parser.hpp
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

#include "cosm/arena/config/arena_map_config.hpp"
#include "cosm/foraging/config/xml/blocks_parser.hpp"
#include "cosm/ds/config/xml/grid2D_parser.hpp"
#include "cosm/repr/config/xml/nests_parser.hpp"

#include "cosm/cosm.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class arena_map_parser
 * \ingroup arena config xml
 *
 * \brief Parses XML parameters for \ref arena_map into \ref arena_map_config.
 */
class arena_map_parser final : public rer::client<arena_map_parser>,
                               public rconfig::xml::xml_config_parser {
 public:
  using config_type = arena_map_config;

  arena_map_parser(void) : ER_CLIENT_INIT("cosm.arena.config.xml.arena_map_parser") {}

  /**
   * \brief The root tag that all arena map parameters should lie under in the
   * XML tree.
   */
  static inline const std::string kXMLRoot = "arena_map";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(cold, pure);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::shared_ptr<config_type>     m_config{nullptr};
  cds::config::xml::grid2D_parser  m_grid{};
  cfconfig::xml::blocks_parser     m_blocks{};
  crepr::config::xml::nests_parser m_nests{};
  /* clang-format on */
};

NS_END(xml, config, arena, cosm);

