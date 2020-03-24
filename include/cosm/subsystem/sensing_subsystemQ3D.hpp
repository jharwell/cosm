/**
 * \file sensing_subsystemQ3D.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_SUBSYSTEM_SENSING_SUBSYSTEMQ3D_HPP_
#define INCLUDE_COSM_SUBSYSTEM_SENSING_SUBSYSTEMQ3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant.hpp>
#include <map>
#include <typeindex>

#include "rcppsw/types/timestep.hpp"

#include "cosm/subsystem/base_sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class sensing_subsystemQ3D
 * \ingroup subsystem
 *
 * \brief The sensing subsystem for all sensors used by robot controllers that
 * actuate in 2D, but can sense in 3D, and as such are "quasi" 3D. Think wheeled
 * robots moving up hills/ramps.
 */
class sensing_subsystemQ3D : public base_sensing_subsystem {
 public:
  /**
   * \param sensors Map of handles to sensing devices, indexed by typeid.
   */
  sensing_subsystemQ3D(const hal::sensors::position_sensor& pos,
                       const sensor_map& sensors)
      : base_sensing_subsystem(pos, sensors) {}

  ~sensing_subsystemQ3D(void) override = default;

  /**
   * \brief Get the robot's current location.
   */
  const rmath::vector3d& position(void) const { return m_position; }
  const rmath::vector3u& discrete_position(void) const { return m_dposition; }

  /**
   * \brief Get the robot's current azimuth heading; this effectively is the
   * angle of the 2D projection of the robots current position in 3D space onto
   * the XY plane.
   */
  const rmath::radians& azimuth(void) const { return m_azimuth; }

  /**
   * \brief Get the robot's current inclination heading; this effectively is the
   * angle the robots current position vector makes with the XY plane.
   */
  const rmath::radians& inclination(void) const { return m_inclination; }

  /**
   * \brief Update the current time and position information for the robot.
   */
  void update(const rtypes::timestep& t,
              const rtypes::discretize_ratio& ratio) override {
    tick(t);
    auto reading = pos_sensor()->reading();
    m_prev_position = m_position;
    m_position = reading.position;
    m_dposition = rmath::dvec2uvec(m_position, ratio.v());
    auto sphere = m_position.to_spherical();
    m_azimuth = sphere.azimuth();
    m_inclination = sphere.inclination();
  }

  /**
   * \brief Get how far the robot has traveled in the last timestep, as well as
   * the direction/magnitude.
   */
  rmath::vector3d tick_travel(void) const {
    return m_position - m_prev_position;
  }

 private:
  /* clang-format off */
  rmath::vector3d               m_position{};
  rmath::vector3d               m_prev_position{};
  rmath::radians                m_azimuth{};
  rmath::radians                m_inclination{};
  rmath::vector3u               m_dposition{};
  /* clang-format off */
};

NS_END(subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_SENSING_SUBSYSTEMQ3D _HPP_ */
