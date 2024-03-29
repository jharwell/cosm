/**
 * \file grid3D_parser.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#include "cosm/ds/config/grid3D_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ds::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class grid3D_parser
 * \ingroup ds config xml
 *
 * \brief Parses XML parameters for \ref cads::arena_grid grid structures into \ref
 * grid3D_config.
 */

class grid3D_parser : public rer::client<grid3D_parser>,
                      public rconfig::xml::xml_config_parser {
 public:
  using config_type = grid3D_config;

  grid3D_parser(void) : ER_CLIENT_INIT("cosm.ds.config.xml.grid3D_parser") {}

  /**
   * \brief The root tag that all grid parameters should lie under in the
   * XML tree.
   */
  static inline const std::string kXMLRoot = "grid3D";

  bool validate(void) const override RCPPSW_PURE;
  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<grid3D_config> m_config{nullptr};
  /* clang-format on */
};

} /* namespace cosm::ds::config::xml */

