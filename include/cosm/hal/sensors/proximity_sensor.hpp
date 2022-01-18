/**
 * \file proximity_sensor.hpp
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

#ifndef INCLUDE_COSM_HAL_SENSORS_PROXIMITY_SENSOR_HPP_
#define INCLUDE_COSM_HAL_SENSORS_PROXIMITY_SENSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <limits>
#include <boost/optional.hpp>

#include "cosm/hal/hal.hpp"

#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
#include "cosm/hal/argos/sensors/ir_sensor.hpp"
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
#include "cosm/hal/ros/sensors/lidar_sensor.hpp"
#endif

#include "rcppsw/math/range.hpp"
#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/hal/sensors/config/proximity_sensor_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
using proximity_sensor_impl = chargos::sensors::ir_sensor;
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
using proximity_sensor_impl = chros::sensors::lidar_sensor;
#endif /* COSM_HAL_TARGET */

/**
 * \class proximity_sensor
 * \ingroup hal sensors
 *
 * \brief Proxmity sensor wrapper to provide a uniform interface to sensing "is
 * there something near me" regardless of robot and platform. Provides some
 * additional higher level functionality beyond raw sensor readings too.
 */
class proximity_sensor final : public rer::client<proximity_sensor>,
                               public proximity_sensor_impl {
 public:
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
  template <typename TSensor>
    proximity_sensor(TSensor * const sensor,
                    const config::proximity_sensor_config* const config)
       : ER_CLIENT_INIT("cosm.hal.sensors.proximity"),
         proximity_sensor_impl(sensor),
         mc_config(*config) {
    enable();
  }
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
   explicit proximity_sensor(const config::proximity_sensor_config* const config)
       : ER_CLIENT_INIT("cosm.hal.sensors.proximity"),
         mc_config(*config) {
     enable();
   }
#endif

  const proximity_sensor& operator=(const proximity_sensor&) = delete;
  proximity_sensor(const proximity_sensor&) = default;

  /**
   * \brief Return the average object reading within proximity for the
   * robot. Proximity is defined as within the "go straight" angle range for the
   * robot
   *
   * If there are not enough objects meeting this criteria that are close enough
   * such that the average distance to them is > than the provided delta,
   * nothing is returned.
   */
  boost::optional<rmath::vector2d> avg_prox_obj(void) const {
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    rmath::vector2d accum;

    size_t count = 0;

    /* constructed, not returned, so we need a copy to iterate over */
    auto readings = this->readings();
    for (auto& r : readings) {
      accum += r;
      ++count;
    }

    if (count == 0) {
      return boost::none;
    }

    accum /= count;

    if (mc_config.fov.contains(accum.angle()) &&
        accum.length() <= mc_config.delta) {
      return boost::none;
    } else {
      return boost::make_optional(accum);
    }
  }
 private:
  /* clang-format off */
  const config::proximity_sensor_config mc_config;
  /* clang-format on */
};

NS_END(sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORS_PROXIMITY_SENSOR_HPP_ */
