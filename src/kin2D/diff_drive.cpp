/**
 * \file diff_drive.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/kin2D/diff_drive.hpp"

#include "cosm/kin2D/config/diff_drive_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, kin2D);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
diff_drive::diff_drive(const config::diff_drive_config* const config,
                       chactuators::diff_drive_actuator&& actuator)
    : ER_CLIENT_INIT("cosm.kin2D.diff_drive"),
      m_config(*config),
      m_fsm(config->max_speed, config->soft_turn_max),
      m_actuator(std::move(actuator)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void diff_drive::fsm_drive(const ckin::twist& delta) {
  m_fsm.change_velocity(delta);

  /* don't need to normalize--done by fsm internally */
  rmath::range<rmath::radians> range(-m_config.soft_turn_max,
                                     m_config.soft_turn_max);

  m_actuator.set_from_twist(m_fsm.configured_twist(), range, m_config.max_speed);
} /* fsm_drive() */

NS_END(kin2D, cosm);
