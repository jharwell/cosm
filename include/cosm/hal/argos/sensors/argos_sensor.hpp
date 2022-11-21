/**
 * \file base_sensor.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/sensors/base_sensor.hpp"
#include "cosm/hal/sensors/identify.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::argos::sensors {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class argos_sensor
 * \ingroup hal argos sensors
 *
 * \brief Base sensor class to provide a common interface to all ARGoS sensors.
 */
template <typename TSensor>
class argos_sensor : public rer::client<argos_sensor<TSensor>>,
                     public chsensors::base_sensor<TSensor*> {
 public:
  using chsensors::base_sensor<TSensor*>::decoratee;

  explicit argos_sensor(TSensor* sensor)
      : ER_CLIENT_INIT("cosm.hal.sensors.argos_sensor"),
        chsensors::base_sensor<TSensor*>(std::move(sensor)) {}

  ~argos_sensor(void) override = default;

  argos_sensor(const argos_sensor&) = delete;
  argos_sensor& operator=(const argos_sensor&) = delete;
  argos_sensor(argos_sensor&&) = default;
  argos_sensor& operator=(argos_sensor&&) = default;

  void enable(void) override {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    decoratee()->Enable();
  }

  void disable(void) override {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    reset();
    decoratee()->Disable();
  }

  void reset(void) override {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    decoratee()->Reset();
  }

  bool is_enabled(void) const override { return decoratee()->IsEnabled(); }
};

} /* namespace cosm::hal::argos::sensors */
