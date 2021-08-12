/**
 * \file diff_drive_actuator.hpp
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

#ifndef INCLUDE_COSM_HAL_ACTUATORS_DIFF_DRIVE_ACTUATOR_HPP_
#define INCLUDE_COSM_HAL_ACTUATORS_DIFF_DRIVE_ACTUATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/hal/hal.hpp"

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#endif /* COSM_HAL_TARGET */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace argos {
class CCI_DifferentialSteeringActuator;
class CCI_PiPuckDifferentialDriveActuator;
} /* namespace argos */

NS_START(cosm, hal, actuators, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TSensor>
using is_argos_generic_ds_actuator = std::is_same<TSensor,
                                                  argos::CCI_DifferentialSteeringActuator>;

template<typename TSensor>
using is_argos_pipuck_ds_actuator = std::is_same<TSensor,
                                                 argos::CCI_PiPuckDifferentialDriveActuator>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class diff_drive_actuator_impl
 * \ingroup hal actuators
 *
 * \brief Differential drive actuator wrapper.
 *
 * Supports the following robots:
 *
 * - ARGoS footbot
 * - ARGoS epuck
 * - ARgoS pipuck
 *
* \tparam TActuator The underlying actuator handle type abstracted away by the
 *                  HAL.
 */
template <typename TActuator>
class diff_drive_actuator_impl : public rer::client<diff_drive_actuator_impl<TActuator>>,
                                 private rpdecorator::decorator<TActuator*> {
 private:
  using rpdecorator::decorator<TActuator*>::decoratee;

 public:
  using impl_type = TActuator;
  /**
   * \brief Construct the wrapper actuator.
   *
   * \param wheels The underlying actuator. If NULL, then that effectively
   *               disables the actuator, and therefore subsequently calling any
   *               member function will have no effect.
   */
  explicit diff_drive_actuator_impl(TActuator* const wheels)
      : ER_CLIENT_INIT("cosm.hal.actuators.diff_drive"),
        rpdecorator::decorator<TActuator*>(wheels) {}

  const diff_drive_actuator_impl& operator=(const diff_drive_actuator_impl&) = delete;
  diff_drive_actuator_impl(const diff_drive_actuator_impl&) = default;

  /**
   * \brief Set the wheel speeds for the current timestep for a footbot/epuck
   * robot. Bounds checking is not performed.
   *
   * \param left Desired speed in m/s for the left wheel.
   * \param right Desired speed in m/s for the right wheel.
   */
  template <typename U = TActuator,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_generic_ds_actuator<U>::value)>
  void set_wheel_speeds(double left, double right) {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);

    /*
     * ARGoS expects velocities to be specified in cm/s, so we have to convert
     * from SI units.
     */
    decoratee()->SetLinearVelocity(left * 100.0, right * 100.0);
  }

  /**
   * \brief Set the wheel speeds for the current timestep for a pipuck
   * robot. Bounds checking is not performed.
   */
  template <typename U = TActuator,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_pipuck_ds_actuator<U>::value)>
  void set_wheel_speeds(double left, double right) {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    decoratee()->SetTargetVelocityLeft(left);
    decoratee()->SetTargetVelocityRight(right);
  }

  /**
   * \brief Stop the wheels of a robot. As far as I know, this is an immediate
   * stop (i.e. no rampdown).
   */
  void reset(void) { set_wheel_speeds(0.0, 0.0); }

  /**
   * \brief Disable the actuator. In ARGoS actuators can't be disabled, so this
   * does nothing.
   */
  template <typename U = TActuator,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_pipuck_ds_actuator<U>::value ||
                                  detail::is_argos_generic_ds_actuator<U>::value)>
  void disable(void) { }
};

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
using diff_drive_actuator = diff_drive_actuator_impl<argos::CCI_DifferentialSteeringActuator>;
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)
using diff_drive_actuator = diff_drive_actuator_impl<argos::CCI_PiPuckDifferentialDriveActuator>;
#endif /* COSM_HAL_TARGET */

NS_END(actuators, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ACTUATORS_DIFF_DRIVE_ACTUATOR_IMPL_HPP_ */
