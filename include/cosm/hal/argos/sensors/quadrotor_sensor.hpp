/**
 * \file quadrotor_sensor.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_DRONE)
#include <argos3/plugins/robots/drone/control_interface/ci_drone_flight_system_sensor.h>
#endif /* COSM_HAL_TARGET */

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/hal/argos/sensors/argos_sensor.hpp"
#include "cosm/kin/twist.hpp"
#include "cosm/hal/argos/sensors/detail/identify.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::argos::sensors {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class quadrotor_sensor_impl
 * \ingroup hal argos sensors
 *
 * \brief Quadrotor "sensor" wrapper. No such sensor actually exists in ARGoS,
 * so we steal the information we need from the quadrotor actuator.
 *
 * Supports the following robots:
 *
 * - ARGoS drone
 *
 * \tparam TSensor The underlying sensor handle type abstracted away by the
 *                 HAL. If nullptr, then that effectively disables the sensor.
 */
template <typename TSensor>
class quadrotor_sensor_impl final : public rer::client<quadrotor_sensor_impl<TSensor>>,
                                    public chargos::sensors::argos_sensor<TSensor> {
 private:
  using chargos::sensors::argos_sensor<TSensor>::decoratee;

 public:
  using impl_type = TSensor;
  using chargos::sensors::argos_sensor<impl_type>::enable;
  using chargos::sensors::argos_sensor<impl_type>::disable;
  using chargos::sensors::argos_sensor<impl_type>::reset;
  using chargos::sensors::argos_sensor<impl_type>::is_enabled;

  explicit quadrotor_sensor_impl(impl_type* const sensor)
      : ER_CLIENT_INIT("cosm.hal.argos.sensors.quadrotor"),
        chargos::sensors::argos_sensor<impl_type>(sensor) {}

  /* move only constructible/assignable for use with saa subsystem */
  const quadrotor_sensor_impl& operator=(const quadrotor_sensor_impl&) = delete;
  quadrotor_sensor_impl(const quadrotor_sensor_impl&) = delete;
   quadrotor_sensor_impl& operator=(quadrotor_sensor_impl&&) = default;
  quadrotor_sensor_impl(quadrotor_sensor_impl&&) = default;

  /**
   * \brief Get the current differential drive reading for the robot.
   */
  ckin::twist reading(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);
    ckin::twist ret;

    ret.linear = rmath::vector3d(decoratee()->GetVelocity().GetX(),
                                 decoratee()->GetVelocity().GetY(),
                                 decoratee()->GetVelocity().GetZ());
    ret.angular = rmath::vector3d(decoratee()->GetAngularVelocity().GetX(),
                                  decoratee()->GetAngularVelocity().GetY(),
                                  decoratee()->GetAngularVelocity().GetZ());
    return ret;
  }

 private:
};

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_DRONE)
using quadrotor_sensor = quadrotor_sensor_impl<::argos::CCI_DroneFlightSystemSensor>;
#endif /* COSM_HAL_TARGET */

} /* namespace cosm::hal::argos::sensors */
