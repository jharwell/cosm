/**
 * \file footbot_saa_subsystem.hpp
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

#ifndef INCLUDE_COSM_ROBOTS_FOOTBOT_FOOTBOT_SAA_SUBSYSTEM_HPP_
#define INCLUDE_COSM_ROBOTS_FOOTBOT_FOOTBOT_SAA_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/robots/footbot/footbot_sensing_subsystem.hpp"
#include "cosm/robots/footbot/footbot_actuation_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, robots, footbot);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class footbot_saa_subsystem
 * \ingroup robots footbot
 *
 * \brief Sensing and Actuation (SAA) subsystem for the footbot robot when it is
 * operating in 3D environments.
 */
class footbot_saa_subsystem final : public subsystem::saa_subsystemQ3D,
                                    public rer::client<footbot_saa_subsystem> {
 public:
  footbot_saa_subsystem(const hal::sensors::position_sensor& pos,
                        const subsystem::sensing_subsystemQ3D::sensor_map& sensors,
                        const subsystem::actuation_subsystem2D::actuator_map& actuators,
                        const steer2D::config::force_calculator_config* steer_config);

  /*
   * 2D BOID interface. We report velocities, speeds, and positions that respect
   * the robot's current Z vector; that is, within the plane defined by the
   * robot's current inclination angle.
   */
  rmath::vector2d linear_velocity(void) const override;
  double angular_velocity(void) const override RCSW_PURE;
  double max_speed(void) const override RCSW_PURE;
  rmath::vector2d pos2D(void) const override RCSW_PURE;

  /**
   * \brief Apply the summed steering forces; change wheel speeds. Resets the
   * summed forces.
   */
  void steer_force2D_apply(void) override;

  footbot_sensing_subsystem* sensing(void) override {
    return static_cast<footbot_sensing_subsystem*>(
        saa_subsystemQ3D::sensing());
  }

  const footbot_sensing_subsystem* sensing(void) const override {
    return static_cast<const footbot_sensing_subsystem*>(
        saa_subsystemQ3D::sensing());
  }

  footbot_actuation_subsystem* actuation(void) override {
    return static_cast<footbot_actuation_subsystem*>(
        saa_subsystemQ3D::actuation());
  }

  const footbot_actuation_subsystem* actuation(void) const override {
    return static_cast<const footbot_actuation_subsystem*>(
        saa_subsystemQ3D::actuation());
  }
};

NS_END(footbot, robots, cosm);

#endif /* INCLUDE_COSM_ROBOTS_FOOTBOT_FOOTBOT_SAA_SUBSYSTEM_HPP_ */
