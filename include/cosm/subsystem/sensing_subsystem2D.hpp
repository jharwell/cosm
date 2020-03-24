/**
 * \file sensing_subsystem2D.hpp
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

#ifndef INCLUDE_COSM_SUBSYSTEM_SENSING_SUBSYSTEM2D_HPP_
#define INCLUDE_COSM_SUBSYSTEM_SENSING_SUBSYSTEM2D_HPP_

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
 * \class sensing_subsystem2D
 * \ingroup subsystem
 *
 * \brief The sensing subsystem for all sensors used by robotics controllers
 * that operate in 2D.
 */
class sensing_subsystem2D : public base_sensing_subsystem {
 public:
  /**
   * \param sensors Map of handles to sensing devices, indexed by typeid.
   */
  sensing_subsystem2D(const hal::sensors::position_sensor& pos,
                      const sensor_map& sensors)
      : base_sensing_subsystem(pos, sensors) {}

  ~sensing_subsystem2D(void) override = default;

  /**
   * \brief Get the robot's current location.
   */
  const rmath::vector2d& position(void) const { return m_position; }
  const rmath::vector2u& discrete_position(void) const { return m_dposition; }

  /**
   * \brief Get the angle of the current robot's heading. A shortcut to help
   * reduce the ache in my typing fingers.
   */
  const rmath::radians& heading(void) const { return m_heading; }

  void update(const rtypes::timestep& t,
              const rtypes::discretize_ratio& ratio) override {
    tick(t);
    auto reading = pos_sensor()->reading();
    m_prev_position = m_position;
    m_position = reading.position.project_on_xy();
    m_dposition = rmath::dvec2uvec(m_position, ratio.v());
    m_heading = reading.z_ang;
  }

  /**
   * \brief Get how far the robot has traveled in the last timestep, as well as
   * the direction/magnitude.
   */
  rmath::vector2d tick_travel(void) const {
    return m_position - m_prev_position;
  }

 private:
  /* clang-format off */
  rmath::vector2d               m_position{};
  rmath::vector2d               m_prev_position{};
  rmath::radians                m_heading{};
  rmath::vector2u               m_dposition{};
  /* clang-format off */
};

NS_END(subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_SENSING_SUBSYSTEM2D _HPP_ */
