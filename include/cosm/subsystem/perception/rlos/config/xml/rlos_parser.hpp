/**
 * \file rlos_parser.hpp
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

#include "rcppsw/config/xml/xml_config_parser.hpp"

#include "cosm/subsystem/perception/rlos/config/rlos_config.hpp"
#include "cosm/subsystem/perception/rlos/config/xml/grid_parser.hpp"
#include "cosm/subsystem/perception/rlos/config/xml/fov_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem::perception::rlos::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class rlos_parser
 * \ingroup subsystem perception rlos config xml
 *
 * \brief Parses XML parameters relating to \ref rlos_config.
 */
class rlos_parser : public rer::client<rlos_parser>,
                    public rconfig::xml::xml_config_parser {
 public:
  using config_type = rlos_config;

  rlos_parser(void)
      : ER_CLIENT_INIT("cosm.subsystem.perception.config.rlos.xml.rlos_parser") {}

  /**
   * \brief The root tag that all rlos parameters should lie under in the
   * XML tree.
   */
  static inline const std::string kXMLRoot = "rlos";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

 private:
  /* clang-format off */
  std::shared_ptr<config_type> m_config{nullptr};
  fov_parser                   m_fov{};
  grid_parser                  m_grid{};
  /* clang-format on */
};

} /* namespace cosm::subsystem::perception::rlos::config::xml */
