/**
 * \file diff_drive_actuator.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
#include "rcppsw/spatial/euclidean_dist.hpp"

#include "cosm/hal/argos/actuators/argos_actuator.hpp"
#include "cosm/hal/actuators/locomotion_actuator.hpp"
#include "cosm/kin/twist.hpp"
#include "cosm/hal/hal.hpp"
#include "cosm/hal/argos/actuators/detail/identify.hpp"

#if (defined(COSM_HAL_TARGET_HAS_DIFF_DRIVE_ACTUATOR) &&        \
     (COSM_HAL_TARGET != COSM_HAL_TARGET_ARGOS_PIPUCK))
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#elif (defined(COSM_HAL_TARGET_HAS_DIFF_DRIVE_ACTUATOR) &&        \
       (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK))
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#endif /* COSM_HAL_TARGET */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::argos::actuators {

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
 * - ARGoS foot-bot
 * - ARGoS e-puck
 * - ARgoS pi-puck
 *
* \tparam TActuator The underlying actuator handle type abstracted away by the
 *                  HAL.
 */
template <typename TActuator>
class diff_drive_actuator_impl : public rer::client<diff_drive_actuator_impl<TActuator>>,
                                 public chargos::actuators::argos_actuator<TActuator>,
                                 public chactuators::locomotion_actuator {
 private:
  using chargos::actuators::argos_actuator<TActuator>::decoratee;

 public:
  /**
   * \enum twist_translate_mode
   *
   * \brief How the \ref ckin::twist should be translated into wheel speeds.
   */
  enum twist_translate_mode {
    /**
     * \brief Use the mode from the ARgoS examples: a heuristic "fudge factor"
     * to go from angular twist in Z -> wheel speeds.
     */
    ekFUDGE,

    /**
     * \brief Use MATH! That is, use conventional kinematics equations to
     * translate.
     *
     * Specifically:
     *
     * \f$ v_l = v_x - (\omega * \frac{l}{2})\f$
     * \f$ v_r = v_x + (\omega * \frac{l}{2})\f$
     *
     * where \f$\omega\f$ is the commanded aangular velocity, and \f$l\f$ is the
     * inter-wheel distance.
     */
    ekMATH
  };

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
  diff_drive_actuator_impl(TActuator* const wheels,
                           const twist_translate_mode& mode,
                           double max_velocity,
                           const rspatial::euclidean_dist& axle_length)
      : ER_CLIENT_INIT("cosm.hal.argos.actuators.diff_drive"),
        chargos::actuators::argos_actuator<TActuator>(wheels),
        mc_mode(mode),
        mc_max_velocity(max_velocity),
        mc_axle_length(axle_length) {}

  /* move only constructible/assignable for use with saa subsystem */
  const diff_drive_actuator_impl& operator=(const diff_drive_actuator_impl&) = delete;
  diff_drive_actuator_impl(const diff_drive_actuator_impl&) = delete;
  diff_drive_actuator_impl& operator=(diff_drive_actuator_impl&&) = default;
  diff_drive_actuator_impl(diff_drive_actuator_impl&&) = default;

  /* locomotion actuator overrides */
  void stop(void) override { reset(); }
  double max_velocity(void) const override { return mc_max_velocity; }

  /**
   * \brief Stop the wheels of a robot. As far as I know, this is an immediate
   * stop (i.e. no rampdown).
   */
  void reset(void) override {
    set_direct(0.0, 0.0);
    argos_actuator<TActuator>::reset();
  }


  void set_from_twist(const ckin::twist& desired,
                      const rmath::range<rmath::radians>& soft_turn) {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    auto speeds = to_wheel_speeds(desired, soft_turn);
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
      const rmath::range<rmath::radians>& soft_turn) {
    std::pair<double, double> speeds;

    /* Both wheels go straight, but one is faster than the other */
    auto angle = rmath::radians(twist.angular.z()).signed_normalize();
    ER_TRACE("Linear_x=%f, angular_z=%f, soft_turn=%s",
             twist.linear.x(),
             angle.v(),
             rcppsw::to_string(soft_turn).c_str());
    if (soft_turn.contains(angle)) {
      speeds = translate_twist(twist, soft_turn);
    } else { /* turn in place */
      speeds.first = -mc_max_velocity;
      speeds.second = mc_max_velocity;

      /*
       * If you don't do this then agents always turn the same direction for all
       * hard turns, rather than the one that minimizes the amount of turning.
       */
      if (rmath::radians(twist.angular.z()) > rmath::radians::kZERO) {
        /* Turn Left */
      } else {
        /* Turn Right */
        std::swap(speeds.first, speeds.second);
        /* return speeds; */
      }
    }
    return speeds;
  } /* to_wheel_speeds() */

 private:
  /**
   * \brief Translate the commanded twist into left/right wheel speeds for soft
   * turns.
   */
  std::pair<double, double> translate_twist(
      const ckin::twist& twist,
      const rmath::range<rmath::radians>& soft_turn) {
    double speed1, speed2;

    if (twist_translate_mode::ekFUDGE == mc_mode) {
      auto soft_turn_max = soft_turn.span() / 2.0;
      double speed_factor = (soft_turn_max.v() - std::fabs(twist.angular.z()) /
                             soft_turn_max.v());
      speed_factor = std::fabs(speed_factor);
      speed1 = twist.linear.x() - twist.angular.z() * (1.0 - speed_factor);
      speed2 = twist.linear.x() + twist.angular.z() * (1.0 - speed_factor);

    } else if (twist_translate_mode::ekMATH == mc_mode) {
      speed1 = twist.linear.x() - (twist.angular.z() * mc_axle_length.v() / 2.0);
      speed2 = twist.linear.x() + (twist.angular.z() * mc_axle_length.v() / 2.0);
    } else {
      ER_FATAL_SENTINEL("Bad twist_translate_mode: %d", mc_mode);
      return {};
    }
    return {speed1, speed2};
  }

  /* clang-format off */
  const twist_translate_mode     mc_mode;
  const double                   mc_max_velocity;
  const rspatial::euclidean_dist mc_axle_length;
  /* clang-format on */
};

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)

using diff_drive_actuator = diff_drive_actuator_impl<::argos::CCI_DifferentialSteeringActuator>;

#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)

using diff_drive_actuator = diff_drive_actuator_impl<::argos::CCI_DifferentialSteeringActuator>;

#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)

using diff_drive_actuator = diff_drive_actuator_impl<::argos::CCI_PiPuckDifferentialDriveActuator>;

#else
struct diff_drive_actuator{};
#endif /* COSM_HAL_TARGET */

} /* namespace cosm::hal::argos::actuators */
