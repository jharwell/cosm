/**
 * \file quadrotor_actuator.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>
#include <functional>
#include "rcppsw/er/client.hpp"

#include "cosm/hal/argos/actuators/argos_actuator.hpp"
#include "cosm/hal/actuators/locomotion_actuator.hpp"
#include "cosm/hal/argos/actuators/detail/identify.hpp"
#include "cosm/kin/pose.hpp"
#include "cosm/kin/twist.hpp"
#include "cosm/hal/argos/sensors/position_sensor.hpp"

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_DRONE)
#include <argos3/plugins/robots/drone/control_interface/ci_drone_flight_system_actuator.h>
#endif /* COSM_HAL_TARGET */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::argos::actuators {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class quadrotor_actuator_impl
 * \ingroup hal argos actuators
 *
 * \brief Quadrotor actuator wrapper
 *
 *  Supports the following robots:
 *
 * - ARGoS drone (controls robot by \c speed, not position).
 *
 * \tparam TActuator The underlying actuator handle type abstracted away by the
 *                   HAL.
 */
template<typename TActuator>
class quadrotor_actuator_impl : public rer::client<quadrotor_actuator_impl<TActuator>>,
                                public chargos::actuators::argos_actuator<TActuator>,
                                public chactuators::locomotion_actuator {
 private:
  using chargos::actuators::argos_actuator<TActuator>::decoratee;

 public:
  using impl_type = TActuator;
  using chargos::actuators::argos_actuator<impl_type>::enable;
  using chargos::actuators::argos_actuator<impl_type>::disable;
  using chargos::actuators::argos_actuator<impl_type>::is_enabled;

  /**
   * \brief Construct the wrapper.
   *
   * We need a way to get the current pose, because this actuator works by being
   * command to a given position, rather than by setting velocities. So, to STOP
   * motion, we need to set our current position to where we currently are at.
   */

  quadrotor_actuator_impl(TActuator* const quadrotors,
                          chargos::sensors::position_sensor&& pos_sensor,
                          double max_velocity)
      : ER_CLIENT_INIT("cosm.hal.argos.actuators.quadrotor"),
        chargos::actuators::argos_actuator<TActuator>(quadrotors),
        mc_max_velocity(max_velocity),
        mc_initial(pos_sensor.reading()) {
    ER_DEBUG("Initial position=%s,zrot=%s",
             rcppsw::to_string(mc_initial.position).c_str(),
             rcppsw::to_string(mc_initial.orientation.z()).c_str());
  }

  /* move only constructible/assignable for use with saa subsystem */
  const quadrotor_actuator_impl& operator=(const quadrotor_actuator_impl&) = delete;
  quadrotor_actuator_impl(const quadrotor_actuator_impl&) = delete;
  quadrotor_actuator_impl& operator=(quadrotor_actuator_impl&&) = default;
  quadrotor_actuator_impl(quadrotor_actuator_impl&&) = default;

  /* locomotion actuator overrides */
  void stop(void) override {
    set_from_pose(mc_initial);
    /* don't call reset()--that will move the drone down to the ground... */
  }
  double max_velocity(void) const override { return mc_max_velocity; }

  void set_from_pose(const ckin::pose& desired) {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    ER_TRACE("Set target: pos=%s, yaw=%s",
             rcppsw::to_string(desired.position).c_str(),
             rcppsw::to_string(desired.orientation.z()).c_str());
    auto pos = desired.position - mc_initial.position;
    auto ang = desired.orientation.z() - mc_initial.orientation.z();
    decoratee()->SetTargetPosition(::argos::CVector3(pos.x(),
                                                     pos.y(),
                                                     pos.z()));
    /** \todo Fix this. Currently does not work because the drone quadrotor is
     * position, not velocity controlled, and setting this naively like I did
     * for 2D differential drive isn't right.
     */
    /* decoratee()->SetTargetYawAngle(desired.orientation.z().v()); */
  }

 private:
  /* clang-format off */
  const double                      mc_max_velocity;
  const ckin::pose                  mc_initial;
  /* clang-format off */
};

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_DRONE)
using quadrotor_actuator = quadrotor_actuator_impl<::argos::CCI_DroneFlightSystemActuator>;
#endif /* COSM_HAL_TARGET */

} /* namespace cosm::hal::argos::actuators */
