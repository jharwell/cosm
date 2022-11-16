/**
 * \file block_motion_parser.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "cosm/foraging/config/block_motion_config.hpp"

#include "cosm/cosm.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_motion_parser
 * \ingroup foraging config xml
 *
 * \brief Parses XML parameters related to block redistribution by the \ref
 * block_motion.
 */
class block_motion_parser : public rer::client<block_motion_parser>,
                            public rconfig::xml::xml_config_parser {
 public:
  using config_type = block_motion_config;

  block_motion_parser(void) : ER_CLIENT_INIT("cosm.foraging.config.xml.block_motion_parser") {}

  /**
   * \brief The root tag that all block motion parameters should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "motion";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_COLD;

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(xml, config, foraging, cosm);

