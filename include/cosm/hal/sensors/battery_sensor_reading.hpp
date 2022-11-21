/**
 * \file battery_sensor_reading.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::sensors {

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
   * \brief In seconds.
   */
  double time_left{-1};
};


} /* namespace cosm::hal::sensors */
