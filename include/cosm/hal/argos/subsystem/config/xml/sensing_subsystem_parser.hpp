 /**
 * \file sensing_subsystem_parser.hpp
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

#include "cosm/hal/sensors/config/xml/proximity_sensor_parser.hpp"
#include "cosm/hal/sensors/config/xml/env_sensor_parser.hpp"
#include "cosm/hal/subsystem/config/sensing_subsystem_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::argos::subsystem::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class sensing_subsystem_parser
 * \ingroup hal argos subsystem config xml
 *
 * \brief Parses XML parameters relating to sensing into \ref
 * sensing_subsystem_config.
 */
class sensing_subsystem_parser final : public rer::client<sensing_subsystem_parser>,
                                       public rconfig::xml::xml_config_parser {
 public:
  using config_type = chsubsystem::config::sensing_subsystem_config;

  sensing_subsystem_parser(void)
      : ER_CLIENT_INIT("cosm.hal.argos.subsystem.config.xml.sensing_subsystem_parser") {}

  ~sensing_subsystem_parser(void) override = default;

  /**
   * \brief The root tag that all robot sensing subsystem parameters should
   * lie under in the XML tree.
   */
  static inline const std::string kXMLRoot = "sensing_subsystem";

  bool validate(void) const override RCPPSW_PURE;
  void parse(const ticpp::Element& node) override RCPPSW_COLD;

  std::string xml_root(void) const override { return kXMLRoot; }

  void env_detection_add(const std::string& target) {
    m_env.detection_add(target);
  }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>                    m_config{nullptr};
  chsensors::config::xml::proximity_sensor_parser m_proximity{};
  chsensors::config::xml::env_sensor_parser       m_env{};
  /* clang-format on */
};

} /* namespace cosm::hal::argos::subsystem::config::xml */
