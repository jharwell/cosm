/**
 * \file diff_drive.hpp
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
#include "rcppsw/er/client.hpp"

#include "cosm/cosm.hpp"
#include "cosm/hal/actuators/diff_drive_actuator.hpp"
#include "cosm/kin2D/diff_drive_fsm.hpp"
#include "cosm/kin2D/config/diff_drive_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, kin2D);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \class diff_drive
 * \ingroup kin2D
 *
 * \brief Model for a TWO wheeled diff drive robot, providing a number of
 * operating modes:
 *
 * \ref drive_type::kTankDrive
 * \ref drive_type::kFSMDrive
 */
class diff_drive : public rer::client<diff_drive> {
 public:
  /**
   * \param type The drive type; see \ref drive_type
   * \param actuator The underlying differential steering actuator (via HAL)
   * \param config Configuration.
   */
  diff_drive(const config::diff_drive_config* config,
             chactuators::diff_drive_actuator&& actuator);

  /* move only constructible/assignable to work with the saa subsystem */
  diff_drive(diff_drive&&) = default;
  diff_drive& operator=(diff_drive&&) = default;

  /*
   * \brief Updates the configured twist via an FSM and sends twist to the
   * actual diff drive actuator for translation into wheel speeds.
   */
  void fsm_drive(const ckin::twist& delta);

  double max_speed(void) const { return m_config.max_speed; }

 private:
  /* clang-format off */
  config::diff_drive_config           m_config;

  diff_drive_fsm                      m_fsm;
  hal::actuators::diff_drive_actuator m_actuator;
  /* clang-format on */

 public:
  /**
   * \brief Reset the differential drive.
   */
  RCPPSW_WRAP_DECLDEF(reset, m_actuator);
  RCPPSW_WRAP_DECLDEF(disable, m_actuator);
  RCPPSW_WRAP_DECLDEF(enable, m_actuator);
};

NS_END(kin2D, cosm);

