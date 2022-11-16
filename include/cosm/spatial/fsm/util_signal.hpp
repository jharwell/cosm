/**
 * \file util_signal.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/fsm/event.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class util_signal
 * \ingroup spatial fsm
 *
 * \brief Signals that sub-states can return in order to notify their super
 * states that a condition that they do not know how to handle has arisen.
 */
class util_signal : public rpfsm::event_signal {
 public:
  enum type {
    /**
     * Signal sent when a robot has left the nest.
     */
    ekLEFT_NEST = rpfsm::event_signal::type::ekEXTERNAL_SIGNALS,

    /**
     * Signal sent when a robot has entered the nest.
     */
    ekENTERED_NEST,

    /**
     * Signal sent when a robot has dropped a block it has been carrying.
     */
    ekDROPPED_BLOCK,

    /**
     * Signal sent when a robot has mechanically malfunctioned.
     */
    ekMECHANICAL_MALFUNCTION,

    /**
     * Signal sent when a robot has been repaired after a mechanical
     * malfunction.
     */
    ekMECHANICAL_REPAIR,

    /**
     * \brief Applications wishing to defined their own event signals
     * should start here.
     */
    ekEXTERNAL_SIGNALS
  };
};

NS_END(fsm, spatial, cosm);
