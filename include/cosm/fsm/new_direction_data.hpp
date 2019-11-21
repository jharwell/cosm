/**
 * \file new_direction_data.hpp
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

#ifndef INCLUDE_COSM_FSM_NEW_DIRECTION_DATA_HPP_
#define INCLUDE_COSM_FSM_NEW_DIRECTION_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/fsm/event.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct new_direction_data
 * \ingroup fsm
 *
 * \brief An argument that can be passed to an FSM state, containing randomness
 * to inject into robot motion by having them change their direction.
 */
struct new_direction_data : public rpfsm::event_data {
  explicit new_direction_data(const rmath::radians& dir_in) : dir(dir_in) {}

  rmath::radians dir;
};

NS_END(fsm, cosm);

#endif /* INCLUDE_COSM_FSM_NEW_DIRECTION_DATA_HPP_ */
