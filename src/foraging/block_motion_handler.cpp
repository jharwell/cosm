/**
 * \file block_motion_handler.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/block_motion_handler.hpp"

#include "cosm/arena/base_arena_map.hpp"
#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/arena/operations/free_block_pickup.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::foraging {

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
  if (kPolicyNone == mc_config.policy) {
    /* nothing to do */
  } else if (kPolicyRandomWalk == mc_config.policy) {
    random_walk(map);
  } else {
    ER_FATAL_SENTINEL("Bad policy %s", mc_config.policy.c_str());
  }
  return m_n_moved;
} /* move_blocks() */

void block_motion_handler::random_walk(carena::base_arena_map* map) {
  auto free_blocks = map->free_blocks(false);
  for (auto* b : free_blocks) {
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
      caops::free_block_drop_visitor drop_op(
          b, *coord, map->grid_resolution(), carena::locking::ekALL_HELD);
      pickup_op.visit(*map);
      ER_ASSERT(!b->is_out_of_sight() && !b->is_carried_by_robot(),
                "Block%d out of sight or carried by robot",
                b->id().v());

      drop_op.visit(*map);
      ER_ASSERT(!b->is_out_of_sight() && !b->is_carried_by_robot(),
                "Block%d out of sight or carried by robot",
                b->id().v());
      ++m_n_moved;
    }
  } /* for(*b..) */
} /* random_walk() */

boost::optional<rmath::vector2z>
block_motion_handler::free_adjacent_coord(const crepr::sim_block3D* block,
                                          const carena::base_arena_map* map) {
  for (size_t i = 0; i < kMAX_TRIES; ++i) {
    rmath::vector2z step(m_rng->uniform(-1, 1), m_rng->uniform(-1, 1));
    auto new_dloc = block->danchor2D() + step;
    auto new_rloc = rmath::zvec2dvec(new_dloc, map->grid_resolution().v());

    /* Same as previous location--doesn't count */
    if (new_dloc == block->danchor2D()) {
      continue;
    }
    if (!map->placement_conflict(block, new_rloc)) {
      RCPPSW_UNUSED bool in_bounds = new_dloc.x() < map->xdsize() &&
                                     new_dloc.y() < map->ydsize() &&
                                     new_dloc.x() > 0 && new_dloc.y() > 0;
      ER_ASSERT(in_bounds,
                "Random walk location %s outside arena bounds "
                "x=[0-%zu],y=[0-%zu]!",
                rcppsw::to_string(new_dloc).c_str(),
                map->xdsize(),
                map->ydsize());
      RCPPSW_UNUSED auto& cell = map->access<cads::arena_grid::kCell>(new_dloc);
      ER_ASSERT(!cell.state_is_known() || cell.state_is_empty(),
                "Cell@%s not unknown or empty [state=%d]",
                rcppsw::to_string(new_dloc).c_str(),
                cell.fsm().current_state());

      return boost::make_optional(new_dloc);
    }
  } /* for(i..) */
  return boost::none;
} /* free_adjacent_coord() */

} /* namespace cosm::foraging */
