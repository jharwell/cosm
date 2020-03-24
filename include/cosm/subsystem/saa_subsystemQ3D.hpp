/**
 * \file saa_subsystemQ3D.hpp
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

#ifndef INCLUDE_COSM_SUBSYSTEM_SAA_SUBSYSTEMQ3D_HPP_
#define INCLUDE_COSM_SUBSYSTEM_SAA_SUBSYSTEMQ3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/subsystem/actuation_subsystem2D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"
#include "cosm/subsystem/base_saa_subsystem2DQ3D.hpp"

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
 * \brief Sensing and Actuation (SAA) subsystem for robot's operating in 2D but
 * sensing in 3D. Implements the BOID interface, which requires both sensors and
 * actuators.
 */
class saa_subsystemQ3D :  rer::client<saa_subsystemQ3D>,
                          public base_saa_subsystem2DQ3D {
 public:
  saa_subsystemQ3D(
      const hal::sensors::position_sensor& pos_sensor,
      const sensing_subsystemQ3D::sensor_map& sensors,
      const actuation_subsystem2D::actuator_map& actuators,
      const steer2D::config::force_calculator_config* const steer_config)
      : ER_CLIENT_INIT("cosm.subsystem.saa"),
        base_saa_subsystem2DQ3D(steer_config),
        m_actuation(std::make_unique<actuation_subsystem2D>(actuators)),
        m_sensing(std::make_unique<sensing_subsystemQ3D>(pos_sensor, sensors)) {}

  virtual sensing_subsystemQ3D* sensing(void) = 0;
  virtual const sensing_subsystemQ3D* sensing(void) const = 0;
  virtual actuation_subsystem2D* actuation(void) = 0;
  virtual const actuation_subsystem2D* actuation(void) const = 0;

 protected:
  virtual sensing_subsystemQ3D* sensing_impl(void) { return m_sensing.get(); }

  virtual const sensing_subsystemQ3D* sensing_impl(void) const {
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
  std::unique_ptr<actuation_subsystem2D>  m_actuation;
  std::unique_ptr<sensing_subsystemQ3D>   m_sensing;
  /* clang-format on */
};

NS_END(subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_SAA_SUBSYSTEMQ3D_HPP_ */
