/**
 * \file actuation_subsystem_parser.hpp
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

#include "cosm/hal/subsystem/config/actuation_subsystem_config.hpp"
#include "cosm/kin2D/config/xml/diff_drive_parser.hpp"
#include "cosm/apf2D/config/xml/apf_manager_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::argos::subsystem::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class actuation_subsystem_parser
 * \ingroup hal argos subsystem config xml
 *
 * \brief Parses XML parameters for \ref actuation_subsystem into
 * \ref actuation_subsystem_config.
 */
class actuation_subsystem_parser final : public rer::client<actuation_subsystem_parser>,
                                           public rconfig::xml::xml_config_parser {
 public:
  using config_type = actuation_subsystem_config;

  actuation_subsystem_parser(void)
      : ER_CLIENT_INIT("cosm.subsystem.config.xml.actuation_subsystem_parser") {}

  /**
   * \brief The root tag that all  actuation subsystem parameters should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "actuation_subsystem";

  bool validate(void) const override RCPPSW_ATTR(pure, cold);
  void parse(const ticpp::Element& node) override RCPPSW_COLD;

  RCPPSW_COLD std::string xml_root(void) const override { return kXMLRoot; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>            m_config{nullptr};
#if defined(COSM_HAL_TARGET_HAS_DIFF_DRIVE_ACTUATOR)
  ckin2D::config::xml::diff_drive_parser  m_diff_drive{};
#endif
#if defined(COSM_HAL_TARGET_HAS_QUADROTOR_ACTUATOR)
  /* TODO: Fill in */
#endif
  capf2D::config::xml::apf_manager_parser m_apf{};
  /* clang-format on */
};

} /* namespace cosm::hal::argos::subsystem::config::xml */
