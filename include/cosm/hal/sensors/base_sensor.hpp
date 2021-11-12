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

#ifndef INCLUDE_COSM_HAL_SENSORS_BASE_SENSOR_HPP_
#define INCLUDE_COSM_HAL_SENSORS_BASE_SENSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_sensor
 * \ingroup hal sensors
 *
 * \brief Base sensor class to provide a common interface to all sensors.
 */
template <typename TSensor>
class base_sensor : public rer::client<base_sensor<TSensor>>,
                    protected rpdecorator::decorator<TSensor*> {
 public:
  using rpdecorator::decorator<TSensor*>::decoratee;

  explicit base_sensor(TSensor* sensor)
      : ER_CLIENT_INIT("cosm.hal.sensors.base_sensor"),
        rpdecorator::decorator<TSensor*>(sensor) {}

  virtual ~base_sensor(void) = default;

  base_sensor(const base_sensor&) = default;
  base_sensor& operator=(const base_sensor&) = default;
  base_sensor(base_sensor&&) = default;
  base_sensor& operator=(base_sensor&&) = default;

  /**
   * \brief Enable the sensor. Should do nothing if sensor is already
   * enabled.
   */

  void enable(void) {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    decoratee()->Enable();
  }

  /**
   * \brief Disable the sensor. Future commands to the sensor should either
   * throw an error or do nothing (application dependent) until the sensor is
   * re-enabled.Should do nothing if sensor is already disabled.
   */
  void disable(void) {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    decoratee()->Disable();
  }

  /**
   * \brief Reset the sensor to its initialized state.
   */
  void reset(void) {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    decoratee()->Reset();
  }
};

NS_END(sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORS_BASE_SENSOR_HPP_ */
