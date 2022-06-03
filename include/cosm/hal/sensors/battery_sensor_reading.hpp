/**
 * \file battery_sensor_reading.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct battery_sensor_reading
 * \ingroup hal sensors
 *
 * \brief A battery sensor reading. If a given robot/implementation doesn't use
 * a field, don't set it.
 */
struct battery_sensor_reading {
  double voltage{-1};

  /**
   * \brief Should be [0,1].
   */
  double percentage{-1};

  /**
   * \brief In seconds
   */
  double time_left{-1};
};


NS_END(sensors, hal, cosm);
