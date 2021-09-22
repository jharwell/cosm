/**
 * \file conflict_checker.cpp
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
#include "cosm/spatial/conflict_checker.hpp"

#include "cosm/arena/base_arena_map.hpp"
#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/ds/loctree.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/repr/nest.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial);

static conflict_checker::status
nest_conflict(const crepr::base_block3D* const block,
              const crepr::nest& nest,
              const rmath::vector2d& drop_loc);
static conflict_checker::status
cache_conflict(const crepr::base_block3D* const block,
               const carepr::arena_cache* const cache,
               const rmath::vector2d& drop_loc);

static conflict_checker::status
block_conflict(const crepr::base_block3D* const block1,
               const crepr::base_block3D* const block2,
               const rmath::vector2d& drop_loc);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
conflict_checker::status
conflict_checker::placement2D(const carena::base_arena_map* map,
                              const crepr::base_block3D* const block,
                              const rmath::vector2d& loc) {
  /*
   * If the robot is really close to a wall, then dropping a block may make it
   * inaccessible to future robots trying to reach it, due to obstacle avoidance
   * kicking in. This can result in an endless loop if said block is the only
   * one a robot knows about (see FORDYCA#242).
   */
  auto dloc = rmath::dvec2zvec(loc, map->grid_resolution().v());
  auto drop_xspan = rmath::xspan(dloc, block->xdsize());
  auto drop_yspan = rmath::yspan(dloc, block->ydsize());

  bool out_of_bounds = !map->distributable_cellsx().contains(drop_xspan) ||
                       !map->distributable_cellsy().contains(drop_yspan);

  if (out_of_bounds) {
    return { true, true };
  }

  status conflict;

  /*
   * To avoid any potential floating point equality comparison errors, pick the
   * rectangle you pass to the loctree to be larger than the space that would be
   * occupied by the block if it is dropped at the specified location.
   *
   * This is WAY faster than a linear scan.
   */
  rmath::vector2d ll(loc - block->rdims2D() * 2.0);
  rmath::vector2d ur(loc + block->rdims2D() * 2.0);
  std::vector<rtypes::type_uuid> ids;

  ids = map->nloctree()->intersections(rds::make_rtree_box(ll, ur));
  for (auto& id : ids) {
    /*
     * Because we picked a larger than strictly required bounding box, we
     * actually need to check what we get in return for overlap.
     */
    conflict = nest_conflict(block, *map->nest(id), loc);
    RCPPSW_CHECK(!(conflict.x && conflict.y));
  } /* for(&nest..) */

  ids = map->bloctree()->intersections(rds::make_rtree_box(ll, ur));
  for (auto& id : ids) {
    /*
     * Because we picked a larger than strictly required bounding box, we
     * actually need to check what we get in return for overlap.
     */
    conflict = block_conflict(block,
                              map->blocks()[id.v()],
                              loc);
    RCPPSW_CHECK(!(conflict.x && conflict.y));
  } /* for(&id..) */

error:
  return conflict;
} /* placement2D() */

conflict_checker::status
conflict_checker::placement2D(const carena::caching_arena_map* map,
                              const crepr::base_block3D* const block,
                              const rmath::vector2d& loc) {
  status conflict =placement2D(static_cast<const carena::base_arena_map*>(map),
                               block,
                               loc);
  if (conflict.x && conflict.y) {
    return conflict;
  }

  /*
   * If the robot is currently right on the edge of a cache, we can't just drop
   * the block here, as it will overlap with the cache, and robots will think
   * that is accessible, but will not be able to vector to it (not all 4 wheel
   * sensors will report the color of a block). See FORDYCA#233.
   *
   * To avoid any potential floating point equality comparison errors, pick the
   * rectangle you pass to the loctree to be larger than the space that would be
   * occupied by the block if it is dropped at the specified location.
   *
   * This is WAY faster than a linear scan.
   */
  rmath::vector2d ll(loc - block->rdims2D() * 2.0);
  rmath::vector2d ur(loc + block->rdims2D() * 2.0);
  auto ids = map->cloctree()->intersections(rds::make_rtree_box(ll, ur));

  for (auto& id : ids) {
    auto it = std::find_if(map->caches().begin(),
                           map->caches().end(),
                           [&](const auto* c) { return c->id() == id; });
    /*
     * Because we picked a larger than strictly required bounding box, we
     * actually need to check what we get in return for overlap.
     */
    conflict = cache_conflict(block, *it, loc);
    RCPPSW_CHECK(!(conflict.x && conflict.y));
  } /* for(cache..) */

error:
  return conflict;
} /* placement2D() */

conflict_checker::status
conflict_checker::placement2D(const rmath::vector2d& ent1_anchor,
                              const rmath::vector2d& ent1_dims,
                              const crepr::entity2D* const ent2) {
  auto loc_xspan = rmath::xspan(ent1_anchor, ent1_dims.x());
  auto loc_yspan = rmath::yspan(ent1_anchor, ent1_dims.y());
  return { ent2->xrspan().overlaps_with(loc_xspan),
           ent2->yrspan().overlaps_with(loc_yspan) };
} /* placement2D() */

conflict_checker::status
conflict_checker::placement2D(const rmath::vector2d& ent1_anchor,
                              const rmath::vector2d& ent1_dims,
                              const crepr::entity3D* const ent2) {
  auto loc_xspan = rmath::xspan(ent1_anchor, ent1_dims.x());
  auto loc_yspan = rmath::yspan(ent1_anchor, ent1_dims.y());
  return { ent2->xrspan().overlaps_with(loc_xspan),
           ent2->yrspan().overlaps_with(loc_yspan) };
} /* placement2D() */

/*******************************************************************************
 * Free Functions
 ******************************************************************************/
conflict_checker::status nest_conflict(const crepr::base_block3D* const block,
                                       const crepr::nest& nest,
                                       const rmath::vector2d& drop_loc) {
  auto drop_xspan = rmath::xspan(drop_loc, block->xrsize().v());
  auto drop_yspan = rmath::yspan(drop_loc, block->yrsize().v());

  return { nest.xrspan().overlaps_with(drop_xspan),
           nest.yrspan().overlaps_with(drop_yspan) };
} /* block_drop_overlap_with_nest() */

conflict_checker::status block_conflict(const crepr::base_block3D* const block1,
                                        const crepr::base_block3D* const block2,
                                        const rmath::vector2d& drop_loc) {
  auto drop_xspan = rmath::xspan(drop_loc, block1->xrsize().v());
  auto drop_yspan = rmath::yspan(drop_loc, block1->yrsize().v());
  return { block2->xrspan().overlaps_with(drop_xspan),
           block2->yrspan().overlaps_with(drop_yspan) };
} /* block_drop_overlap_with_block() */

conflict_checker::status cache_conflict(const crepr::base_block3D* const block,
                                        const carepr::arena_cache* const cache,
                                        const rmath::vector2d& drop_loc) {
  auto drop_xspan = rmath::xspan(drop_loc, block->xrsize().v());
  auto drop_yspan = rmath::yspan(drop_loc, block->yrsize().v());
  return { cache->xrspan().overlaps_with(drop_xspan),
           cache->yrspan().overlaps_with(drop_yspan) };
} /* block_drop_overlap_with_cache() */

NS_END(spatial, cosm);
