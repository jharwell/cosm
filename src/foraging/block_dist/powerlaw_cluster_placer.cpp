/**
 * \file powerlaw_cluster_placer.cpp
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
#include "cosm/foraging/block_dist/powerlaw_cluster_placer.hpp"

#include <algorithm>
#include <cmath>

#include "cosm/repr/spatial_entity2D.hpp"
#include "cosm/arena/base_arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);
using cosm::arena::ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
powerlaw_cluster_placer::powerlaw_cluster_placer(carena::base_arena_map* map,
                                                 const rmath::vector3d& c_block_bb,
                                                 size_t n_attempts,
                                                 rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.foraging.block_dist.powerlaw_cluster_placer"),
      mc_n_attempts(n_attempts),
      mc_block_bb(c_block_bb),
      m_map(map),
      m_rng(rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
powerlaw_cluster_placer::placement
powerlaw_cluster_placer::placement_guess(const rtypes::type_uuid& c_id,
                                         size_t size,
                                         const rmath::vector3d& c_block_bb) {
  /*
   * If there are blocks which are larger in X/Y than the arena resolution, we
   * need to account for that so that a cluster of size k will ALWAYS be able to
   * hold k blocks, regardless of size.
   */
  size_t xfactor = static_cast<size_t>(c_block_bb.x() / m_map->grid_resolution().v());
  size_t yfactor = static_cast<size_t>(c_block_bb.y() / m_map->grid_resolution().v());

  /*
   * sqrt() might not be an integer, so we round up to get a smooth-ish
   * continuum of cluster sizes.
   */
  auto clust_dim_base = static_cast<size_t>(std::ceil(std::sqrt(size)));

  size_t clust_dim_x = clust_dim_base + xfactor * 2;
  size_t clust_dim_y = clust_dim_base + yfactor * 2;
  rmath::rangez area_xrange(clust_dim_x, m_map->xdsize() - clust_dim_x - 1);
  rmath::rangez area_yrange(clust_dim_y, m_map->ydsize() - clust_dim_y - 1);

  ER_TRACE("Cluster%d size=%zu placement area_xrange=%s,area_yrange=%s",
           c_id.v(),
           size,
           rcppsw::to_string(area_xrange).c_str(),
           rcppsw::to_string(area_yrange).c_str());

  size_t anchor_x = m_rng->uniform(area_xrange);
  size_t anchor_y = m_rng->uniform(area_yrange);

  auto ll = rmath::vector2z(anchor_x, anchor_y);
  auto ur = rmath::vector2z(anchor_x + clust_dim_x, anchor_y + clust_dim_y);
  auto view = m_map->decoratee().layer<arena_grid::kCell>()->subgrid(ll, ur);
  rmath::rangez clust_xrange(view.origin()->loc().x(),
                             view.origin()->loc().x() + view.shape()[0]);
  rmath::rangez clust_yrange(view.origin()->loc().y(),
                             view.origin()->loc().y() + view.shape()[1]);

  ER_TRACE("Cluster%d guess: anchor=%s,xrange=%s,arena_yrange=%s",
           c_id.v(),
           rcppsw::to_string(ll).c_str(),
           rcppsw::to_string(clust_xrange).c_str(),
           rcppsw::to_string(clust_yrange).c_str());

  return { c_id, view, clust_xrange, clust_yrange, size };
} /* placement_guess() */

bool powerlaw_cluster_placer::placement_check(
    const placement& c_guess,
    const placements& c_placed,
    const cds::const_spatial_entity_vector& c_entities) const {
  /* check for placed cluster overlap */
  for (const auto& placement_i : c_placed) {
    if (placement_i.xrange.overlaps_with(c_guess.xrange) &&
        placement_i.yrange.overlaps_with(c_guess.yrange)) {
      ER_TRACE("Placed cluster%d [xrange=%s,yrange=%s] overlaps guessed "
               "cluster%d placement [xrange=%s,yrange=%s]",
               placement_i.id.v(),
               rcppsw::to_string(placement_i.xrange).c_str(),
               rcppsw::to_string(placement_i.yrange).c_str(),
               c_guess.id.v(),
               rcppsw::to_string(c_guess.xrange).c_str(),
               rcppsw::to_string(c_guess.yrange).c_str());
      return false;
    }
  } /* for(&placement_i..) */

  /* check for entity overlap */
  for (const auto* ent : c_entities) {
    if (ent->xdspan().overlaps_with(c_guess.xrange) &&
        ent->ydspan().overlaps_with(c_guess.yrange)) {
      ER_TRACE("Entity%d [xrange=%s,yrange=%s] overlaps guessed cluster%d "
               "placement [xrange=%s,yrange=%s]",
               ent->id().v(),
               rcppsw::to_string(ent->xdspan()).c_str(),
               rcppsw::to_string(ent->ydspan()).c_str(),
               c_guess.id.v(),
               rcppsw::to_string(c_guess.xrange).c_str(),
               rcppsw::to_string(c_guess.yrange).c_str());
      return false;
    }
  } /* for(*ent..) */
  /* check for out of bounds */
  RCPPSW_CHECK(m_map->distributable_cellsx().contains(c_guess.xrange));
  RCPPSW_CHECK(m_map->distributable_cellsy().contains(c_guess.yrange));
  return true;

error:
  return false;
} /* placement_check() */

powerlaw_cluster_placer::placements powerlaw_cluster_placer::operator()(
    const cds::const_spatial_entity_vector& c_entities,
    const std::vector<size_t>& c_sizes) {
  ER_INFO("Computing cluster placements for %zu clusters", c_sizes.size());

  /*
   * Place clusters in arena via guess and check, one at a time, from
   * largest to smallest.
   */
  placements placed;
  for (size_t j = 0; j < c_sizes.size(); ++j) {
    for (size_t i = 0; i < mc_n_attempts; ++i) {
      auto guess = placement_guess(rtypes::type_uuid(j), c_sizes[j], mc_block_bb);
      if (placement_check(guess, placed, c_entities)) {
        placed.push_back(guess);
        ER_DEBUG("Cluster%d capacity=%zu placed: xrange=%s,yrange=%s",
                 guess.id.v(),
                 guess.capacity,
                 rcppsw::to_string(guess.xrange).c_str(),
                 rcppsw::to_string(guess.yrange).c_str());
        break;
      }
    } /* for(i..) */
  } /* for(j..) */

  ER_ASSERT(placed.size() == c_sizes.size(),
            "Only able to place %zu/%zu clusters in arena",
            placed.size(),
            c_sizes.size());
  return placed;
} /* operator()() */

NS_END(block_dist, foraging, cosm);
