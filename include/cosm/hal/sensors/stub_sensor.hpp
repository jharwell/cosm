/**
 * \file stub_sensor.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>

#include "cosm/hal/sensors/base_sensor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::sensors {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class stub_sensor
 * \ingroup hal sensors
 *
 * \brief Stub sensor class to make things compile when a sensor is disabled.
 */
template<typename TReadingType>
class stub_sensor : public chsensors::base_sensor<std::nullptr_t> {
 public:
  using rpdecorator::decorator<std::nullptr_t>::decoratee;
  using impl_type = std::nullptr_t;

  stub_sensor(void) = default;

  explicit stub_sensor(std::nullptr_t&& sensor)
      : base_sensor<impl_type>(std::move(sensor)) {}

  virtual ~stub_sensor(void) = default;

  stub_sensor(const stub_sensor&) = default;
  stub_sensor& operator=(const stub_sensor&) = default;
  stub_sensor(stub_sensor&&) = default;
  stub_sensor& operator=(stub_sensor&&) = default;

  void enable(void) override {}
  void disable(void) override {}
  void reset(void) override {}
  bool is_enabled(void) const override { return false;}
  TReadingType readings(void) const { return {}; }
};

} /* namespace cosm::hal::sensors */
