/**
 * \file base_sensor.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/sensors/base_sensor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, argos, sensors);

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

  virtual ~argos_sensor(void) = default;

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

NS_END(sensors, argos, hal, cosm);

