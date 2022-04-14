/**
 * \file util_signal.hpp
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
