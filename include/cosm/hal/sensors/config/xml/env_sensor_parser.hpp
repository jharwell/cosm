/**
 * \file env_sensor_parser.hpp
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

#include "cosm/hal/sensors/config/env_sensor_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"
#include "cosm/hal/sensors/config/xml/env_sensor_detection_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, sensors, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_sensor_parser
 * \ingroup hal sensors config xml
 *
 * \brief Parses XML parameters relating to HAL env sensor into \ref
 * env_sensor_config.
 */
class env_sensor_parser : public rer::client<env_sensor_parser>,
                             public rconfig::xml::xml_config_parser {
 public:
  using config_type = env_sensor_config;

  env_sensor_parser(void) : ER_CLIENT_INIT("cosm.hal.argos.sensors.config.xml.env_sensor_parser") {}

  ~env_sensor_parser(void) override = default;

  /**
   * \brief The root tag that all robot env sensor parameters should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "env_sensor";

  bool validate(void) const override RCPPSW_PURE;
  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }

  /**
   * \brief Add a detection target name to the list of names which will be
   * parsed from the XML subtree rooted at \ref kXMLRoot.
   */
  void detection_add(const std::string& target) { m_targets.push_back(target); }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type>              m_config{nullptr};
  std::list<std::string>                    m_targets{};
  /* clang-format on */
};

NS_END(xml, config, sensors, hal, cosm);
