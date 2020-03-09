/**
 * \file footbot_saa_subsystem2D.hpp
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

#ifndef INCLUDE_COSM_ROBOTS_FOOTBOT_FOOTBOT_SAA_SUBSYSTEM2D_HPP_
#define INCLUDE_COSM_ROBOTS_FOOTBOT_FOOTBOT_SAA_SUBSYSTEM2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/subsystem/saa_subsystem2D.hpp"
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
 * \class footbot_saa_subsystem2D
 * \ingroup robots footbot
 *
 * \brief Sensing and Actuation (SAA) subsystem for the footbot robot when it is
 * operating in strictly 2D environments.
 */
class footbot_saa_subsystem2D final : public subsystem::saa_subsystem2D,
                                      public rer::client<footbot_saa_subsystem2D> {
 public:
  using sensing_subsystem_type = footbot_sensing_subsystem<subsystem::sensing_subsystem2D>;

  footbot_saa_subsystem2D(const hal::sensors::position_sensor& pos,
                        const subsystem::sensing_subsystem2D::sensor_map& sensors,
                        const subsystem::actuation_subsystem2D::actuator_map& actuators,
                        const steer2D::config::force_calculator_config* steer_config);

  /* BOID interface */
  rmath::vector2d linear_velocity(void) const override;
  double angular_velocity(void) const override RCSW_PURE;
  double max_speed(void) const override RCSW_PURE;
  rmath::vector2d position(void) const override RCSW_PURE;

  /**
   * \brief Apply the summed steering forces; change wheel speeds. Resets the
   * summed forces.
   */
  void steer_force2D_apply(void) override;

  sensing_subsystem_type* sensing(void) override {
    return static_cast<sensing_subsystem_type*>(
        subsystem::saa_subsystem2D::sensing_impl());
  }

  const sensing_subsystem_type* sensing(void) const override {
    return static_cast<const sensing_subsystem_type*>(
        subsystem::saa_subsystem2D::sensing_impl());
  }

  footbot_actuation_subsystem* actuation(void) override {
    return static_cast<footbot_actuation_subsystem*>(
        subsystem::saa_subsystem2D::actuation_impl());
  }

  const footbot_actuation_subsystem* actuation(void) const override {
    return static_cast<const footbot_actuation_subsystem*>(
        subsystem::saa_subsystem2D::actuation_impl());
  }
};

NS_END(footbot, robots, cosm);

#endif /* INCLUDE_COSM_ROBOTS_FOOTBOT_FOOTBOT_SAA_SUBSYSTEM2D_HPP_ */
