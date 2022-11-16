/**
 * \file proximity_sensor_parser.hpp
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

#include "cosm/hal/sensors/config/proximity_sensor_config.hpp"
#include "rcppsw/config/xml/xml_config_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, sensors, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class proximity_sensor_parser
 * \ingroup hal sensors config xml
 *
 * \brief Parses XML parameters relating to HAL proximity sensor into \ref
 * proximity_sensor_config.
 */
class proximity_sensor_parser : public rer::client<proximity_sensor_parser>,
                                public rconfig::xml::xml_config_parser {
 public:
  using config_type = proximity_sensor_config;

  proximity_sensor_parser(void) : ER_CLIENT_INIT("cosm.hal.sensors.config.xml.proximity_sensor_parser") {}

  ~proximity_sensor_parser(void) override = default;

  /**
   * \brief The root tag that all robot proximity sensor parameters should lie
   * under in the XML tree.
   */
  static inline const std::string kXMLRoot = "proximity_sensor";

  bool validate(void) const override RCPPSW_PURE;
  void parse(const ticpp::Element& node) override;

  std::string xml_root(void) const override { return kXMLRoot; }

 private:
  const rconfig::base_config* config_get_impl(void) const override {
    return m_config.get();
  }

  /* clang-format off */
  std::unique_ptr<config_type> m_config{nullptr};
  /* clang-format on */
};

NS_END(xml, config, sensors, hal, cosm);

