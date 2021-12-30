/**
 * \file diff_drive_sensor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_HAL_ARGOS_SENSORS_DIFF_DRIVE_SENSOR_HPP_
#define INCLUDE_COSM_HAL_ARGOS_SENSORS_DIFF_DRIVE_SENSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_sensor.h>
#endif /* COSM_HAL_TARGET */

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/hal/argos/sensors/argos_sensor.hpp"
#include "cosm/kin/twist.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace argos {
class CCI_DifferentialSteeringSensor;
class CCI_PiPuckDifferentialDriveSensor;
} /* namespace argos */

NS_START(cosm, hal, argos, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename T>
using is_generic_ds_sensor = std::is_same<T,
                                          ::argos::CCI_DifferentialSteeringSensor>;
template<typename T>
using is_pipuck_ds_sensor = std::is_same<T,
                                         ::argos::CCI_PiPuckDifferentialDriveSensor>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class diff_drive_sensor_impl
 * \ingroup hal argos sensors
 *
 * \brief Differential drive sensor wrapper.
 *
 * Supports the following robots:
 *
 * - ARGoS footbot
 * - ARGoS epuck
 * - ARGoS pipuck
 *
 * \tparam TSensor The underlying sensor handle type abstracted away by the
 *                 HAL. If nullptr, then that effectively disables the sensor.
 */
template <typename TSensor>
class diff_drive_sensor_impl final : public rer::client<diff_drive_sensor_impl<TSensor>>,
                                  public chargos::sensors::argos_sensor<TSensor> {
 private:
  using chargos::sensors::argos_sensor<TSensor>::decoratee;

 public:
  using impl_type = TSensor;
  using chargos::sensors::argos_sensor<impl_type>::enable;
  using chargos::sensors::argos_sensor<impl_type>::disable;
  using chargos::sensors::argos_sensor<impl_type>::reset;
  using chargos::sensors::argos_sensor<impl_type>::is_enabled;

  struct wheel_reading {
    /* The LINEAR (not angular) velocity of the wheel (m/s) */
    double vel{};
  };

  struct raw_sensor_reading {
    wheel_reading left{};
    wheel_reading right{};
    double axle_length{}; /* in units of m */
  };

  explicit diff_drive_sensor_impl(impl_type* const sensor)
      : ER_CLIENT_INIT("cosm.hal.sensors.diff_drive"),
        chargos::sensors::argos_sensor<impl_type>(sensor) {}

  const diff_drive_sensor_impl& operator=(const diff_drive_sensor_impl&) = delete;
  diff_drive_sensor_impl(const diff_drive_sensor_impl&) = default;

  /**
   * \brief Get the current differential drive reading for the robot.
   */
  template <typename U = impl_type,
            RCPPSW_SFINAE_DECLDEF(detail::is_generic_ds_sensor<U>::value)>
  ckin::twist reading(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    auto tmp = decoratee()->GetReading();

    /*
     * ARGoS reports the distances and velocities in cm and cm/s for some
     * reason, so put it in SI units (meters), like a sane person.
     */
    auto raw = raw_sensor_reading{wheel_reading{tmp.VelocityLeftWheel / 100.0},
                                  wheel_reading{tmp.VelocityRightWheel / 100.0},
                                  tmp.WheelAxisLength / 100.0};

    return twist_calc(raw);
  }

  /**
   * \brief Get the current differential drive reading for the robot.
   */
  template <typename U = impl_type,
            RCPPSW_SFINAE_DECLDEF(detail::is_pipuck_ds_sensor<U>::value)>
  ckin::twist reading(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    auto raw = raw_sensor_reading{wheel_reading{decoratee()->GetLeftVelocity()},
                                  wheel_reading{decoratee()->GetRightVelocity()},
                                  0.0};

    return twist_calc(raw);
  }

 private:
  static ckin::twist twist_calc(const raw_sensor_reading& raw) {
    ckin::twist ret;
    ret.angular = rmath::vector3d::Z * (raw.left.vel - raw.right.vel) / raw.axle_length;
    ret.linear = rmath::vector3d::X * (raw.left.vel + raw.right.vel) / 2.0;
    return ret;
  }
};

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
using diff_drive_sensor = diff_drive_sensor_impl<::argos::CCI_DifferentialSteeringSensor>;
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)
using diff_drive_sensor = diff_drive_sensor_impl<::argos::CCI_PiPuckDifferentialDriveSensor>;
#else
class diff_drive_sensor{};
#endif /* COSM_HAL_TARGET */

NS_END(sensors, argos, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ARGOS_SENSORS_DIFF_DRIVE_SENSOR_HPP_ */
