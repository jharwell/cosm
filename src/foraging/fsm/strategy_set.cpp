/**
 * \file strategy_set.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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
#include "cosm/foraging/fsm/strategy_set.hpp"

#include "cosm/spatial/strategy/nest/acq/base_acq.hpp"
#include "cosm/spatial/strategy/nest/exit/base_exit.hpp"
#include "cosm/spatial/strategy/explore/base_explore.hpp"
#include "cosm/spatial/strategy/blocks/drop/base_drop.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, foraging, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
strategy_set::strategy_set(void) = default;

strategy_set::strategy_set(std::unique_ptr<cssexplore::base_explore> explore_in,
                           std::unique_ptr<cssnest::acq::base_acq> nest_acq_in,
                           std::unique_ptr<cssnest::exit::base_exit> nest_exit_in,
                           std::unique_ptr<cssblocks::drop::base_drop> block_drop_in)
    : explore(std::move(explore_in)),
      nest_acq(std::move(nest_acq_in)),
      nest_exit(std::move(nest_exit_in)),
      block_drop(std::move(block_drop_in)) {}


strategy_set::strategy_set(strategy_set&& other)
    : explore(std::move(other.explore)),
      nest_acq(std::move(other.nest_acq)),
      nest_exit(std::move(other.nest_exit)),
      block_drop(std::move(other.block_drop)) {}

strategy_set::~strategy_set(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

NS_END(fsm, foraging, cosm);
