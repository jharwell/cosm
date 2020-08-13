/**
 * \file block_motion_handler.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include "cosm/foraging/block_motion_handler.hpp"

#include "cosm/arena/operations/free_block_pickup.hpp"
#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/spatial/conflict_checker.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/arena/base_arena_map.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, foraging);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_motion_handler::block_motion_handler(
    const config::block_motion_config* const config,
    rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.foraging.block_motion_handler"),
      mc_config(*config),
      m_rng(rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
size_t block_motion_handler::move_blocks(carena::base_arena_map* map) {
  m_n_moved = 0;
  if (kPolicyNull == mc_config.policy) {
    /* nothing to do */
  } else if (kPolicyRandomWalk == mc_config.policy) {
    random_walk(map);
  } else {
    ER_FATAL_SENTINEL("Bad policy %s", mc_config.policy.c_str());
  }
  return m_n_moved;
} /* move_blocks() */

void block_motion_handler::random_walk(carena::base_arena_map* map) {
  auto free_blocks = map->free_blocks();
  for (auto *b : free_blocks) {
    if (!m_rng->bernoulli(mc_config.random_walk_prob)) {
      continue;
    }
    if (auto coord = free_adjacent_coord(b, map)) {
      ER_INFO("Move block%d@%s/%s to %s",
              b->id().v(),
              rcppsw::to_string(b->danchor2D()).c_str(),
              rcppsw::to_string(b->ranchor2D()).c_str(),
              rcppsw::to_string(*coord).c_str());
      auto pickup_op = caops::free_block_pickup_visitor::by_arena(b);

      /*
       * Not really holding any locks, but this function always called in
       * non-concurrent contexts, so no need to grab locks.
       */
      caops::free_block_drop_visitor drop_op(b,
                                             *coord,
                                             map->grid_resolution(),
                                             carena::arena_map_locking::ekALL_HELD);
      pickup_op.visit(*map);
      drop_op.visit(*map);
      ++m_n_moved;
    }
  } /* for(*b..) */
} /* random_walk() */

boost::optional<rmath::vector2z> block_motion_handler::free_adjacent_coord(
    const crepr::base_block3D* block,
    const carena::base_arena_map* map) {
  for (size_t i = 0; i < kMAX_TRIES; ++i) {
    rmath::vector2z step(m_rng->uniform(-1, 1),
                         m_rng->uniform(-1, 1));
    auto new_dloc = block->danchor2D() + step;
    auto new_rloc = rmath::zvec2dvec(new_dloc, map->grid_resolution().v());

    /* Same as previous location--doesn't count */
    if (new_dloc == block->danchor2D()) {
        continue;
    }
    if (!map->placement_conflict(block, new_rloc)) {
      RCSW_UNUSED bool in_bounds = new_dloc.x() < map->xdsize() &&
                       new_dloc.y() < map->ydsize() &&
          new_dloc.x() > 0 && new_dloc.y() > 0;
      ER_ASSERT(in_bounds,
                "Random walk location %s outside arena bounds x=[0-%zu],y=[0-%zu]!",
                rcppsw::to_string(new_dloc).c_str(),
                map->xdsize(),
                map->ydsize());
      RCSW_UNUSED auto& cell = map->access<cds::arena_grid::kCell>(new_dloc);
      ER_ASSERT(!cell.state_is_known() || cell.state_is_empty(),
                "Cell@%s not unknown or empty [state=%d]",
                rcppsw::to_string(new_dloc).c_str(),
                cell.fsm().current_state());

      return boost::make_optional(new_dloc);
    }
  } /* for(i..) */
  return boost::none;
} /* free_adjacent_coord() */

NS_END(foraging, cosm);
