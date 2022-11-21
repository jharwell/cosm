/**
 * \file fov_parser.hpp
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

#include "cosm/subsystem/perception/rlos/config/fov_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem::perception::rlos::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class fov_parser
 * \ingroup subsystem perception rlos config xml
 *
 * \brief Parses XML parameters relating to \ref fov_config.
 */
class fov_parser : public rer::client<fov_parser>,
                    public rconfig::xml::xml_config_parser {
 public:
  using config_type = fov_config;

  fov_parser(void)
      : ER_CLIENT_INIT("cosm.subsystem.perception.config.rlos.xml.fov_parser") {}

  /**
   * \brief The root tag that all rlos parameters should lie under in the
   * XML tree.
   */
  static inline const std::string kXMLRoot = "fov";

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

} /* namespace cosm::subsystem::perception::rlos::config::xml */
