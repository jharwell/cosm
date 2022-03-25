/**
 * \file diagnostics.hpp
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
NS_START(cosm, hal, actuators);

/*******************************************************************************
 * Enum Definitions
 ******************************************************************************/
/**
 * \enum diagnostics
 * \ingroup hal actuators
 *
 * \brief Different types of things that robots can signal in some way (e.g.,
 * via LEDs) to give insight into their internal state when debugging.
 */
enum diagnostics {
  /**
   * \brief A robot is exploring for something.
   */
  ekEXPLORE,

  /**
   * \brief A robot has successfully finished something.
   */
  ekSUCCESS,

  /**
   * \brief A robot is taxiing somewhere.
   */
  ekTAXIS,

  /**
   * \brief A robot is leaving the nest.
   */
  ekLEAVING_NEST,

  /**
   * \brief A robot is wait for a signal.
   */
  ekWAIT_FOR_SIGNAL,
  /**
   * \brief A robot has a goal and is moving towards it.
   */
  ekVECTOR_TO_GOAL,
  /**
   * \brief A robot is experiencing interference.
   */
  ekEXP_INTERFERENCE,

  ekMAX
};

NS_END(actuators, hal, cosm);
