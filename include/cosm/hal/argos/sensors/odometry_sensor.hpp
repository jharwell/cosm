/**
 * \file odometry_sensor.hpp
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

#include "rcppsw/math/vector3.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/kin/odometry.hpp"
#include "cosm/hal/argos/sensors/position_sensor.hpp"
#if defined(COSM_HAL_TARGET_HAS_DIFF_DRIVE_SENSOR)
#include "cosm/hal/argos/sensors/diff_drive_sensor.hpp"
#elif defined(COSM_HAL_TARGET_HAS_QUADROTOR_SENSOR)
#include "cosm/hal/argos/sensors/quadrotor_sensor.hpp"
#endif

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::argos::sensors {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class odometry_sensor_impl
 * \ingroup hal argos sensors
 *
 * \brief Odometry sensor built from the position and diff drive sensors.
 *
 * Supports the following robots:
 *
 * - ARGoS foot-bot
 * - ARGoS e-puck
 * - ARGoS pi-puck
*/
template <typename TPoseSrc, typename TTwistSrc>
class odometry_sensor_impl final
    : public rer::client<odometry_sensor_impl<TPoseSrc, TTwistSrc>>,
      public chsensors::base_sensor<std::pair<TPoseSrc, TTwistSrc> > {
 public:
  using decoratee_type = std::pair<TPoseSrc, TTwistSrc>;
  using pose_src_type = typename decoratee_type::first_type;
  using twist_src_type = typename decoratee_type::second_type;

 private:
  using chsensors::base_sensor<decoratee_type>::decoratee;

 public:
  using chsensors::base_sensor<decoratee_type>::enable;
  using chsensors::base_sensor<decoratee_type>::disable;
  using chsensors::base_sensor<decoratee_type>::reset;
  using chsensors::base_sensor<decoratee_type>::is_enabled;

  odometry_sensor_impl(pose_src_type&& pose, twist_src_type&& twist)
      : ER_CLIENT_INIT("cosm.hal.argos.sensors.odometry"),
        chsensors::base_sensor<decoratee_type>(
            std::make_pair(std::move(pose),
                           std::move(twist))) {
    /* odometry is almost always needed, and its cheap to obtain from ARGoS */
    enable();
  }

  /* move only constructible/assignable for use with saa subsystem */
  const odometry_sensor_impl& operator=(const odometry_sensor_impl&) = delete;
  odometry_sensor_impl(const odometry_sensor_impl&) = delete;
  odometry_sensor_impl& operator=(odometry_sensor_impl&&) = default;
  odometry_sensor_impl(odometry_sensor_impl&&) = default;

  /**
   * \brief Get the current odometry sensor readings for the robot.
   *
   * \note If a robot is currently executing a HARD turn/spinning in place, then
   *       the reported linear velocity will be 0, which can make some virtual
   *       forces not work without special cases.
   *
   * \return A \ref ckin::odometry reading.
   */
  ckin::odometry reading(void) const {
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    ckin::odometry ret;

    /* get pose */
    ret.pose = decoratee().first.reading();

    /* get twist */
    ret.twist = decoratee().second.reading();

    return ret;
  }

  void enable(void) override {
    decoratee().first.enable();
    decoratee().second.enable();
  }
  void disable(void) override {
    decoratee().first.disable();
    decoratee().second.disable();
  }
  void reset(void) override {
    decoratee().first.reset();
    decoratee().second.reset();
  }

  bool is_enabled(void) const override {
    return decoratee().first.is_enabled() && decoratee().second.is_enabled();
  }
};

#if defined(COSM_HAL_TARGET_HAS_DIFF_DRIVE_SENSOR)
using odometry_sensor = odometry_sensor_impl<position_sensor, diff_drive_sensor>;
#elif defined(COSM_HAL_TARGET_HAS_QUADROTOR_SENSOR)
using odometry_sensor = odometry_sensor_impl<position_sensor, quadrotor_sensor>;
#endif /* COSM_HAL_TARGET_HAS_DIFF_DRIVE_SENSOR */

} /* namespace cosm::hal::argos::sensors */
