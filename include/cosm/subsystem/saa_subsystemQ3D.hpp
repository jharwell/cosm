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

#ifndef INCLUDE_COSM_SUBSYSTEM_SAA_SUBSYSTEMQ3D_HPP_
#define INCLUDE_COSM_SUBSYSTEM_SAA_SUBSYSTEMQ3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <utility>

#include "cosm/steer2D/force_calculator.hpp"
#include "cosm/subsystem/actuation_subsystem2D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem);
class actuation_subsystem2D;
class sensing_subsystemQ3D;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class saa_subsystemQ3D
 * \ingroup subsystem
 *
 * \brief Sensing and Actuation (SAA) subsystem for 2D and quasi-3D (Q3D) robots
 */
class saa_subsystemQ3D : public steer2D::boid {
 public:
  using sensing_type = sensing_subsystemQ3D;
  using actuation_type = actuation_subsystem2D;

  explicit saa_subsystemQ3D(
      const hal::sensors::position_sensor& pos_sensor,
      const sensing_subsystemQ3D::sensor_map& sensors,
      const actuation_subsystem2D::actuator_map& actuators,
      const steer2D::config::force_calculator_config* const steer_config)
      : m_steer2D_calc(*this, steer_config),
        m_actuation(std::make_unique<actuation_type>(actuators)),
        m_sensing(std::make_unique<sensing_type>(pos_sensor, sensors)) {}

  /*
   * These functions are overridable in order to allow derived classes to return
   * pointers to sensing/actuation subsystems covariant with the types passed to
   * this class as template arguments.
   */
  virtual sensing_type* sensing(void) { return m_sensing.get(); }
  virtual const sensing_type* sensing(void) const { return m_sensing.get(); }
  virtual actuation_type* actuation(void) { return m_actuation.get(); }
  virtual const actuation_type* actuation(void) const {
    return m_actuation.get();
  }

  /**
   * \brief Apply the summed steering forces; change wheel speeds. Should reset
   * the summed forces.
   */
  virtual void steer_force2D_apply(void) = 0;

  const steer2D::force_calculator& steer_force2D(void) const {
    return m_steer2D_calc;
  }
  steer2D::force_calculator& steer_force2D(void) { return m_steer2D_calc; }

 protected:
  void actuation(std::unique_ptr<actuation_type> actuation) {
    m_actuation = std::move(actuation);
  }
  void sensing(std::unique_ptr<sensing_type> sensing) {
    m_sensing = std::move(sensing);
  }

 private:
  /* clang-format off */
  steer2D::force_calculator       m_steer2D_calc;
  std::unique_ptr<actuation_type> m_actuation;
  std::unique_ptr<sensing_type>   m_sensing;
  /* clang-format on */
};

NS_END(subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_SAA_SUBSYSTEMQ3D_HPP_ */
