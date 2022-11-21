/**
 * \file nests_parser.hpp
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

#include "rcppsw/config/xml/xml_config_parser.hpp"

#include "cosm/repr/config/nests_config.hpp"
#include "cosm/repr/config/xml/nest_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class nests_parser
 * \ingroup repr config xml
 *
 * \brief Parses XML parameters for related to \ref nest objects into
 * \ref nest_config.
 */
class nests_parser : public rer::client<nests_parser>,
                     public rconfig::xml::xml_config_parser {
 public:
  using config_type = nests_config;

  nests_parser(void) : ER_CLIENT_INIT("cosm.repr.config.xml.nests_parser") {}

  /**
   * \brief The root tag that all nest parameters should lie under in the
   * XML tree.
   */
  static inline const std::string kXMLRoot = "nests";

  void parse(const ticpp::Element& node) override RCPPSW_COLD;
  bool validate(void) const override RCPPSW_ATTR(pure, cold);

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<nests_config> m_config{nullptr};
  nest_parser                   m_nest{};
  /* clang-format on */
};

} /* namespace cosm::repr::config::xml */

