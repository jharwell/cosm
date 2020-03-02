/**
 * \file arena_block_op_visit_set.hpp
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

#ifndef INCLUDE_COSM_FORAGING_EVENTS_ARENA_BLOCK_OP_VISIT_SET_HPP_
#define INCLUDE_COSM_FORAGING_EVENTS_ARENA_BLOCK_OP_VISIT_SET_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/mpl/typelist.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class base_block2D;
} // namespace cosm::repr

namespace cosm::foraging::ds {
class arena_map;
} // namespace ds

NS_START(cosm, foraging, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \ingroup foraging events
 *
 * \brief Interface specifying the core class of classes any action involving
 * dropping a block will need to visit (think data structures).
 */
using arena_block_op_visit_typelist =
    rmpl::typelist<cfds::arena_map, crepr::base_block2D>;

NS_END(events, foraging, cosm);

#endif /* INCLUDE_COSM_EVENTS_FORAGING_ARENA_BLOCK_OP_VISIT_SET_HPP_ */
