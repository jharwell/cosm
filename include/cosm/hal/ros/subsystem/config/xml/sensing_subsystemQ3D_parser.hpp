 /**
 * \file sensing_subsystemQ3D_parser.hpp
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
#include "cosm/hal/ros/sensors/config/xml/sonar_sensor_parser.hpp"
#include "cosm/hal/subsystem/config/sensing_subsystemQ3D_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, ros, subsystem, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class sensing_subsystemQ3D_parser
 * \ingroup hal ros subsystem config xml
 *
 * \brief Parses XML parameters relating to sensing into \ref
 * sensing_subsystemQ3D_config.
 */
class sensing_subsystemQ3D_parser final : public rer::client<sensing_subsystemQ3D_parser>,
                                          public rconfig::xml::xml_config_parser {
 public:
  using config_type = sensing_subsystemQ3D_config;

  sensing_subsystemQ3D_parser(void)
      : ER_CLIENT_INIT("cosm.hal.ros.subsystem.config.xml.sensing_subsystemQ3D_parser") {}

  ~sensing_subsystemQ3D_parser(void) override = default;

  /**
   * \brief The root tag that all robot sensing subsystem parameters should
   * lie under in the XML tree.
   */
  static inline const std::string kXMLRoot = "sensing_subsystemQ3D";

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
  chrsensors::config::xml::sonar_sensor_parser    m_sonar{};
  /* clang-format on */
};

NS_END(xml, config, subsystem, ros, hal, cosm);
