/**
 * \file blocks_parser.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "rcppsw/control/config/xml/waveform_parser.hpp"

#include "cosm/foraging/config/blocks_config.hpp"
#include "cosm/cosm.hpp"
#include "cosm/foraging/config/xml/block_dist_parser.hpp"
#include "cosm/foraging/config/xml/block_motion_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class blocks_parser
 * \ingroup foraging config xml
 *
 * \brief Parses XML parameters related to blocks in the arena into \ref
 * blocks_config.
 */
class blocks_parser : public rer::client<blocks_parser>,
                      public rconfig::xml::xml_config_parser {
 public:
  using config_type = blocks_config;

  blocks_parser(void) : ER_CLIENT_INIT("cosm.foraging.config.xml.blocks_parser") {}

  /**
   * \brief The root tag that all block parameters should lie under in the
   * XML tree.
   */
  static inline const std::string kXMLRoot = "blocks";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  block_dist_parser            m_dist{};
  block_motion_parser          m_motion{};
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

} /* namespace cosm::foraging::config::xml */

