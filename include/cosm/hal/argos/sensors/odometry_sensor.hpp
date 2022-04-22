/**
 * \file odometry_sensor.hpp
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

#include "rcppsw/math/vector3.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/cosm.hpp"
#include "cosm/hal/argos/sensors/position_sensor.hpp"
#include "cosm/hal/argos/sensors/diff_drive_sensor.hpp"
#include "cosm/hal/hal.hpp"
#include "cosm/kin/odometry.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, argos, sensors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class odometry_sensor
 * \ingroup hal argos sensors
 *
 * \brief Odometry sensor built from the position and diff drive sensors.
 *
 * Supports the following robots:
 *
 * - ARGoS footbot
 * - ARGoS epuck
 * - ARGoS pipuck
*/
class odometry_sensor final
    : public rer::client<odometry_sensor>,
      public chsensors::base_sensor<std::pair<position_sensor,
                                                  diff_drive_sensor>
                                        > {
 public:
  using decoratee_type = std::pair<position_sensor, diff_drive_sensor>;

 private:
  using chsensors::base_sensor<decoratee_type>::decoratee;

 public:
  using chsensors::base_sensor<decoratee_type>::enable;
  using chsensors::base_sensor<decoratee_type>::disable;
  using chsensors::base_sensor<decoratee_type>::reset;
  using chsensors::base_sensor<decoratee_type>::is_enabled;

  explicit odometry_sensor(position_sensor&& position,
                           diff_drive_sensor&& steering)
      : ER_CLIENT_INIT("cosm.hal.argos.sensors.odometry"),
        chsensors::base_sensor<decoratee_type>(
            std::make_pair(std::move(position),
                           std::move(steering))) {}

  /* move only constructible/assignable for use with saa subsystem */
  const odometry_sensor& operator=(const odometry_sensor&) = delete;
  odometry_sensor(const odometry_sensor&) = delete;
  odometry_sensor& operator=(odometry_sensor&&) = default;
  odometry_sensor(odometry_sensor&&) = default;

  /**
   * \brief Get the current odometry sensor readings for the robot.
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

    /*
     * If speed comes back as 0.0, then we are executing a hard turn, probably
     * as we vector somewhere. In order to have the arrival force work properly,
     * we need to have a velocity with a non-zero length and the correct heading
     * angle at all times. So we report that we have velocity even though we do
     * not, for the purposes of making those calculations work.
     *
     * There probably is a better way to do this, but I don't know what it is.
     */
    if (ret.twist.linear.length() <= std::numeric_limits<double>::epsilon()) {
      ret.twist.linear = rmath::vector3d::X * 0.01;
    }
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

NS_END(sensors, argos, hal, cosm);
