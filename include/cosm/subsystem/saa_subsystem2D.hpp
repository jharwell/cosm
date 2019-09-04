/**
 * @file saa_subsystem2D.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_SUBSYSTEM_SAA_SUBSYSTEM2D_HPP_
#define INCLUDE_COSM_SUBSYSTEM_SAA_SUBSYSTEM2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/steer2D/force_calculator.hpp"
#include "cosm/subsystem/actuation_subsystem2D.hpp"
#include "cosm/subsystem/sensing_subsystem2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class saa_subsystem2D
 * @ingroup cosm subsystem
 *
 * @brief Sensing and Actuation (SAA) subsystem for the robot. Implements the
 * BOID interface, which requires both sensors and actuators.
 */
class saa_subsystem2D : public steer2D::boid,
                        public rer::client<saa_subsystem2D> {
 public:
  saa_subsystem2D(
      sensing_subsystem2D::sensor_map& sensors,
      actuation_subsystem2D::actuator_map& actuators,
      const steer2D::config::force_calculator_config* const steer_config)
      : ER_CLIENT_INIT("cosm.subsystem.saa"),
        m_actuation(std::make_unique<actuation_subsystem2D>(actuators)),
        m_sensing(std::make_unique<sensing_subsystem2D>(sensors)),
        m_steer2D_calc(*this, steer_config) {}

  /**
   * @brief Apply the summed steering forces; change wheel speeds. Resets the
   * summed forces.
   */
  virtual void steer_force2D_apply(void) = 0;

  const steer2D::force_calculator& steer_force2D(void) const {
    return m_steer2D_calc;
  }
  steer2D::force_calculator& steer_force2D(void) { return m_steer2D_calc; }

  virtual sensing_subsystem2D* sensing(void) = 0;
  virtual const sensing_subsystem2D* sensing(void) const = 0;
  virtual actuation_subsystem2D* actuation(void) = 0;
  virtual const actuation_subsystem2D* actuation(void) const = 0;

 protected:
  virtual sensing_subsystem2D* sensing_impl(void) { return m_sensing.get(); }

  virtual const sensing_subsystem2D* sensing_impl(void) const {
    return m_sensing.get();
  }

  virtual actuation_subsystem2D* actuation_impl(void) {
    return m_actuation.get();
  }
  virtual const actuation_subsystem2D* actuation_impl(void) const {
    return m_actuation.get();
  }

 private:
  /* clang-format off */
  std::unique_ptr<actuation_subsystem2D> m_actuation;
  std::unique_ptr<sensing_subsystem2D>   m_sensing;
  steer2D::force_calculator              m_steer2D_calc;
  /* clang-format on */
};

NS_END(subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_SAA_SUBSYSTEM2D_HPP_ */
