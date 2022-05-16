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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/angles.hpp"
#include "rcppsw/math/range.hpp"

#include "cosm/hal/argos/actuators/argos_actuator.hpp"
#include "cosm/kin/twist.hpp"

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

NS_START(cosm, hal, argos, actuators, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TSensor>
using is_generic_ds_actuator = std::is_same<TSensor,
                                                  ::argos::CCI_DifferentialSteeringActuator>;

template<typename TSensor>
using is_pipuck_ds_actuator = std::is_same<TSensor,
                                                 ::argos::CCI_PiPuckDifferentialDriveActuator>;

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
                                 public chargos::actuators::argos_actuator<TActuator> {
 private:
  using chargos::actuators::argos_actuator<TActuator>::decoratee;

 public:
  using impl_type = TActuator;
  using chargos::actuators::argos_actuator<impl_type>::enable;
  using chargos::actuators::argos_actuator<impl_type>::disable;
  using chargos::actuators::argos_actuator<impl_type>::is_enabled;

  /**
   * \brief Construct the wrapper actuator.
   *
   * \param wheels The underlying actuator. If NULL, then that effectively
   *               disables the actuator, and therefore subsequently calling any
   *               member function will have no effect.
   */
  explicit diff_drive_actuator_impl(TActuator* const wheels)
      : ER_CLIENT_INIT("cosm.hal.argos.actuators.diff_drive"),
        chargos::actuators::argos_actuator<TActuator>(wheels) {}

  /* move only constructible/assignable for use with saa subsystem */
  const diff_drive_actuator_impl& operator=(const diff_drive_actuator_impl&) = delete;
  diff_drive_actuator_impl(const diff_drive_actuator_impl&) = delete;
  diff_drive_actuator_impl& operator=(diff_drive_actuator_impl&&) = default;
  diff_drive_actuator_impl(diff_drive_actuator_impl&&) = default;

  /**
   * \brief Stop the wheels of a robot. As far as I know, this is an immediate
   * stop (i.e. no rampdown).
   */
  void reset(void) override {
    set_direct(0.0, 0.0);
    argos_actuator<TActuator>::reset();
  }

  void set_from_twist(const ckin::twist& desired,
                      const rmath::range<rmath::radians>& soft_turn,
                      double max_speed) {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    auto speeds = to_wheel_speeds(desired, soft_turn, max_speed);
    set_direct(speeds.first, speeds.second);
  }

  /**
   * \brief Set the wheel speeds for the current timestep for a footbot/epuck
   * robot. Bounds checking is not performed.
   *
   * \param left Desired speed in m/s for the left wheel.
   * \param right Desired speed in m/s for the right wheel.
   */
  template <typename U = TActuator,
            RCPPSW_SFINAE_DECLDEF(detail::is_generic_ds_actuator<U>::value)>
  void set_direct(double left, double right) {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
             "%s called when disabled",
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
            RCPPSW_SFINAE_DECLDEF(detail::is_pipuck_ds_actuator<U>::value)>
  void set_direct(double left, double right) {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);
    decoratee()->SetTargetVelocityLeft(left);
    decoratee()->SetTargetVelocityRight(right);
  }

  std::pair<double, double> to_wheel_speeds(
      const ckin::twist& twist,
      const rmath::range<rmath::radians>& soft_turn,
      double max_speed) {
    double speed1, speed2;

    /* Both wheels go straight, but one is faster than the other */
    auto angle = rmath::radians(twist.angular.z()).signed_normalize();
    ER_TRACE("Linear_x=%f, angular_z=%f, soft_turn=%s",
             twist.linear.x(),
             angle.v(),
             rcppsw::to_string(soft_turn).c_str());
    if (soft_turn.contains(angle)) {
      auto soft_turn_max = soft_turn.span() / 2.0;
      double speed_factor = (soft_turn_max.v() - std::fabs(twist.angular.z()) /
                             soft_turn_max.v());
      speed_factor = std::fabs(speed_factor);

      speed1 = twist.linear.x() - twist.linear.x() * (1.0 - speed_factor);
      speed2 = twist.linear.x() + twist.linear.x() * (1.0 - speed_factor);
    } else { /* turn in place */
      speed1 = -max_speed;
      speed2 = max_speed;
    }
    if (rmath::radians(twist.angular.z()) > rmath::radians::kZERO) {
      /* Turn Left */
      return {speed1, speed2};
    } else {
      /* Turn Right */
      return {speed2, speed1};
    }
  } /* to_wheel_speeds() */
};

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
using diff_drive_actuator = diff_drive_actuator_impl<::argos::CCI_DifferentialSteeringActuator>;
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)
using diff_drive_actuator = diff_drive_actuator_impl<::argos::CCI_PiPuckDifferentialDriveActuator>;
#endif /* COSM_HAL_TARGET */

NS_END(actuators, argos, hal, cosm);
