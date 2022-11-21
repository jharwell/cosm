/**
 * \file diagnostics.hpp
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
namespace cosm::hal::actuators {

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

} /* namespace cosm::hal::actuators */
