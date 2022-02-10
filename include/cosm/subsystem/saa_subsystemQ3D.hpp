/**
 * \file saa_subsystemQ3D.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include <memory>
#include <utility>

#include "cosm/steer2D/force_calculator.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"
#include "cosm/subsystem/actuation_subsystem2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class saa_subsystemQ3D
 * \ingroup subsystem
 *
 * \brief Sensing and Actuation (SAA) subsystem for the footbot robot when it is
 * operating in 3D environments (sensing in 3D, actuating in 2D). The precise
 * set of sensors/actuators abstracted away at a lower level, so that this class
 * can be used for any robot.
 */
class saa_subsystemQ3D final : public steer2D::boid,
                               public rer::client<saa_subsystemQ3D> {
 public:
  using sensing_type = csubsystem::sensing_subsystemQ3D;
  using actuation_type = csubsystem::actuation_subsystem2D;

  saa_subsystemQ3D(
      sensing_type::sensor_map&& sensors,
      actuation_type::actuator_map&& actuators,
      const steer2D::config::force_calculator_config* const steer_config);

  /*
   * 2D BOID interface. We report velocities, speeds, and positions that respect
   * the robot's current Z vector; that is, within the plane defined by the
   * robot's current zenith angle.
   */
  ckin::odometry odometry(void) const override;
  double max_speed(void) const override RCPPSW_PURE;

  sensing_type* sensing(void) { return m_sensing.get(); }
  const sensing_type* sensing(void) const { return m_sensing.get(); }
  actuation_type* actuation(void) { return m_actuation.get(); }
  const actuation_type* actuation(void) const {
    return m_actuation.get();
  }

  /**
   * \brief Apply the summed steering forces; change wheel speeds. Resets the
   * summed forces.
   */
  void steer_force2D_apply(void);

  const steer2D::force_calculator& steer_force2D(void) const {
    return m_steer2D_calc;
  }
  steer2D::force_calculator& steer_force2D(void) { return m_steer2D_calc; }

 private:
  /* clang-format off */
  std::unique_ptr<actuation_type> m_actuation;
  std::unique_ptr<sensing_type>   m_sensing;
  steer2D::force_calculator       m_steer2D_calc;
  /* clang-format on */
};

NS_END(subsystem, cosm);

