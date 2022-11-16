/**
 * \file sr04us_sensor.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/ros/sensors/sonar_sensor.hpp"

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_ETURTLEBOT3)
#include "sr04us/ping_service.hpp"
#endif /* COSM_HAL_TARGET */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, sensors);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
sonar_sensor::sonar_sensor(const cros::topic& robot_ns,
                           const config::sonar_sensor_config* config)
    : ER_CLIENT_INIT("cosm.hal.ros.sensors.sonar"),
      ros_service_sensor(robot_ns),
      m_config(*config) {
  enable();
  ER_ASSERT(decoratee().exists(),
            "Connected service %s does not exist for %s?",
            service_name().c_str(),
            cpal::kRobotType.c_str());
}

sonar_sensor::sonar_sensor(sonar_sensor&& other)
    : ER_CLIENT_INIT("cosm.hal.ros.sensors.sonar_sensor"),
      ros_service_sensor(other.robot_ns()),
      m_config(other.m_config) {
  enable();
}

sonar_sensor& sonar_sensor::operator=(sonar_sensor&& rhs) {
  this->m_config = rhs.m_config;
  rhs.disable();
  this->enable();
  return *this;
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void sonar_sensor::enable(void) {
  if (is_enabled()) {
    return;
  }
  auto name = robot_ns() / cros::topic(rosbridge::sr04us::kServiceName);
  ER_INFO("%s: ns=%s, name=%s, connect=%s, trig=%d, echo=%d",
          __FUNCTION__,
          robot_ns().c_str(),
          rosbridge::sr04us::kServiceName,
          name.c_str(),
          m_config.trigger_pin,
          m_config.echo_pin);

  connect<rosbridge::sr04us::PingService>(name);
}

std::vector<chsensors::env_sensor_reading> sonar_sensor::readings(void) {
  ER_ASSERT(is_enabled(), "%s called when disabled", __FUNCTION__);

  rosbridge::sr04us::PingService srv;
  srv.request.trig = m_config.trigger_pin;
  srv.request.echo = m_config.echo_pin;

  std::vector<chsensors::env_sensor_reading> ret;

  for (size_t i = 0; i < 10; ++i) {
    if (!decoratee().call(srv)) {
      ER_DEBUG("Failed to receive ping data!");
      continue;
    }
    for (auto& r : srv.response.readings) {
      ER_DEBUG("Received reading: %f", r.value);
      if (r.value > 0) {
        ret.emplace_back(r.value);
      }
    } /* for(&r..) */
    return ret;
  } /* for(i..) */

  ER_WARN("Did not receive ping data after 10 tries");
  return {};
}

NS_END(sensors, ros, hal, cosm);
