/**
 * \file light_sensor.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/ros/sensors/light_sensor.hpp"

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_ETURTLEBOT3)
#include "tsl2591/ReadingsService.h"
#endif /* COSM_HAL_TARGET */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(rosbridge);
namespace tsl2591 = ::tsl2591;
NS_END(rosbridge);

NS_START(cosm, hal, ros, sensors);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
light_sensor::light_sensor(const cros::topic& robot_ns)
    : ER_CLIENT_INIT("cosm.hal.ros.sensors.light"), ros_service_sensor(robot_ns) {
  enable();
  ER_ASSERT(decoratee().exists(),
            "Connected service %s does not exist for %s?",
            service_name().c_str(),
            cpal::kRobotType.c_str());
}

light_sensor::light_sensor(light_sensor&& other)
    : ER_CLIENT_INIT("cosm.hal.ros.sensors.light_sensor"),
      ros_service_sensor(other.robot_ns()) {
  enable();
}

light_sensor& light_sensor::operator=(light_sensor&& rhs) {
  this->m_light = rhs.m_light;
  rhs.disable();
  this->enable();
  return *this;
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void light_sensor::reset(void) { m_light = {}; }

void light_sensor::enable(void) {
  if (is_enabled()) {
    return;
  }
  auto name = robot_ns() / cros::topic(kServiceName);
  ER_INFO("%s: ns=%s, name=%s, connect=%s",
          __FUNCTION__,
          robot_ns().c_str(),
          kServiceName.c_str(),
          name.c_str());

  connect<rosbridge::tsl2591::ReadingsService>(name);
}

std::vector<light_sensor::reading_type> light_sensor::readings(void) {
  ER_ASSERT(is_enabled(), "%s called when disabled", __FUNCTION__);

  rosbridge::tsl2591::ReadingsService srv;

  std::vector<chsensors::light_sensor_reading> ret;

  if (!decoratee().call(srv)) {
    ER_WARN("Failed to receive light sensor data!");
    return ret;
  }

  for (auto& r : srv.response.readings) {
    ER_INFO("Received reading: %f -> %f", r.angle, r.intensity);
    if (r.intensity > 0) {
      ret.push_back({ r.intensity, rmath::radians(r.angle) });
    }
  } /* for(&r..) */
  return ret;
}

NS_END(sensors, ros, hal, cosm);
