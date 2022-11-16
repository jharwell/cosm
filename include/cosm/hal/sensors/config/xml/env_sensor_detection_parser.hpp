/**
 * \file env_sensor_detection_parser.hpp
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
#include <list>

#include "cosm/hal/sensors/config/env_sensor_detection_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, sensors, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_sensor_detection_parser
 * \ingroup hal sensors config xml
 *
 * \brief Parses XML parameters relating to HAL env sensor into \ref
 * env_sensor_detection_config.
 */
class env_sensor_detection_parser : public rer::client<env_sensor_detection_parser>,
                                       public rconfig::xml::xml_config_parser {
 public:
  using config_type = env_sensor_detection_config;

  RCPPSW_COLD explicit env_sensor_detection_parser(const std::string& name)
      : ER_CLIENT_INIT("cosm.hal.argos.sensors.config.xml.env_sensor_detection_parser"),
        m_name(name) {}

  ~env_sensor_detection_parser(void) override = default;

  void parse(const ticpp::Element& node) override RCPPSW_COLD;

  RCPPSW_COLD std::string xml_root(void) const override { return m_name; }

  RCPPSW_COLD void xml_name(const std::string& name) { m_name = name; }

 private:
  RCPPSW_COLD const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::string                  m_name;
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(xml, config, sensors, hal, cosm);
