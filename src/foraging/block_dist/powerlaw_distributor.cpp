/**
 * \file powerlaw_distributor.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "cosm/foraging/block_dist/powerlaw_distributor.hpp"

#include <algorithm>
#include <cmath>
#include <map>

#include "cosm/ds/arena_grid.hpp"
#include "cosm/foraging/config/block_dist_config.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/foraging/block_dist/multi_cluster_distributor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);
using cosm::ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
powerlaw_distributor::powerlaw_distributor(
    const config::powerlaw_dist_config* const config,
    cds::arena_grid* grid,
    rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.foraging.block_dist.powerlaw"),
      base_distributor(grid, rng),
      m_n_clusters(config->n_clusters),
      m_pwrdist(config->pwr_min, config->pwr_max, 2) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
dist_status powerlaw_distributor::distribute_block(crepr::base_block3D* block,
                                                   cds::const_spatial_entity_vector& entities) {
  /*
   * Try to find a block cluster to distribute to, starting from a random
   * cluster size.
   */
  size_t start = rng()->uniform(0UL, m_dists.size() - 1);
  for (size_t i = 0; i < m_dists.size(); ++i) {
    auto* mclust = m_dists[(start + i) % m_dists.size()].get();

    if (mclust->size() == mclust->capacity()) {
      continue;
    }
    ER_DEBUG("Attempt distribution: block%d -> cluster group: capacity%zu,size=%zu]",
             block->id().v(),
             mclust->capacity(),
             mclust->size());

    if (dist_status::ekSUCCESS == mclust->distribute_block(block, entities)) {
      return dist_status::ekSUCCESS;
    } /* for(&dist..) */
  } /* for(i..) */

  ER_FATAL_SENTINEL("Unable to distribute block to any cluster");
  return dist_status::ekFAILURE;
} /* distribute_block() */

bool powerlaw_distributor::initialize(const cds::const_spatial_entity_vector& entities,
                                      const rmath::vector3d& block_bb) {
  /* Compute cluster locations in arena */
  auto placements = cluster_placements_calc(entities, block_bb, m_n_clusters);
  if (!placements) {
    ER_WARN("Unable to compute all cluster placements");
    return false;
  }

  std::map<size_t, std::vector<cds::arena_grid::view>> grids;
  std::for_each(placements->begin(),
                placements->end(),
                [&](const auto& placement) {
                  return grids[placement.capacity].push_back(placement.view);
    });

  for (auto& pair : grids) {
    auto mclust = std::make_unique<multi_cluster_distributor>(pair.second,
                                                              arena_grid(),
                                                              pair.first,
                                                              rng());
    ER_INFO("Mapped multi-cluster: capacity=%zu", mclust->capacity());
    m_dists.push_back(std::move(mclust));
  } /* for(i..) */

  return true;
} /* initialize() */

typename powerlaw_distributor::cluster_placements powerlaw_distributor::
cluster_placements_guess(const std::vector<size_t>& clust_sizes,
                         const rmath::vector3d& block_bb) {
  cluster_placements placements;

  /*
   * If there are blocks which are larger in X/Y than the arena resolution, we
   * need to account for that so that a cluster of size k will ALWAYS be able to
   * hold k blocks, regardless of size.
   */
  size_t xfactor = static_cast<size_t>(block_bb.x() / arena_grid()->resolution().v());
  size_t yfactor = static_cast<size_t>(block_bb.y() / arena_grid()->resolution().v());

  for (size_t i = 0; i < clust_sizes.size(); ++i) {
    /*
     * sqrt() might not be an integer, so we round up to get a smooth-ist
     * continuum of cluster sizes.
     */
    auto clust_dim_base = static_cast<size_t>(std::ceil(std::sqrt(clust_sizes[i])));

    size_t clust_dim_x = clust_dim_base + xfactor * 2;
    size_t clust_dim_y = clust_dim_base + yfactor * 2;
    rmath::rangez area_xrange(clust_dim_x, arena_grid()->xdsize() - clust_dim_x - 1);
    rmath::rangez area_yrange(clust_dim_y, arena_grid()->ydsize() - clust_dim_y - 1);

    ER_DEBUG("Cluster%zu size=%zu placement area_xrange=%s,area_yrange=%s",
             i,
             clust_sizes[i],
             rcppsw::to_string(area_xrange).c_str(),
             rcppsw::to_string(area_yrange).c_str());

    size_t anchor_x = rng()->uniform(area_xrange);
    size_t anchor_y = rng()->uniform(area_yrange);

    auto ll = rmath::vector2z(anchor_x, anchor_y);
    auto ur = rmath::vector2z(anchor_x + clust_dim_x, anchor_y + clust_dim_y);
    auto view = arena_grid()->layer<arena_grid::kCell>()->subgrid(ll, ur);
    rmath::rangez clust_xrange(view.origin()->loc().x(),
                               view.origin()->loc().x() + view.shape()[0]);
    rmath::rangez clust_yrange(view.origin()->loc().y(),
                               view.origin()->loc().y() + view.shape()[1]);

    ER_TRACE("Cluster%zu params: anchor=%s,xrange=%s,arena_yrange=%s",
             i,
             rcppsw::to_string(ll).c_str(),
             rcppsw::to_string(clust_xrange).c_str(),
             rcppsw::to_string(clust_yrange).c_str());

    placements.push_back({rtypes::type_uuid(i),
            view,
            clust_xrange,
            clust_yrange,
            clust_sizes[i]});
  } /* for(i..) */
  return placements;
} /* cluster_placements_guess() */

bool powerlaw_distributor::cluster_placements_check(
    const cluster_placements& placements,
    const cds::const_spatial_entity_vector& entities) {
  /* check for entity overlap */
  for (const auto& placement_i : placements) {
    for (auto *ent : entities) {
      if (placement_i.xrange.overlaps_with(ent->xdspan()) &&
          placement_i.yrange.overlaps_with(ent->ydspan())) {
        ER_TRACE("One or more entities overlap cluster%d placement",
                 placement_i.id.v());
        return false;
      }
    } /* for(*ent..) */
  } /* for(&placement_i..) */

  /* check for inter-cluster placement overlap */
  for (const auto& placement_i : placements) {
    auto placement_overlap_check = [&](const auto& placement_j) {
      /* no overlap possible with self */
      if (placement_j.id == placement_i.id) {
        return false;
      }
      return placement_i.xrange.overlaps_with(placement_j.xrange) &&
      placement_i.yrange.overlaps_with(placement_j.yrange);
    };

    if (std::any_of(placements.begin(),
                    placements.end(),
                    placement_overlap_check)) {
      ER_TRACE("One or more cluster placements overlap cluster%d placement",
               placement_i.id.v());
      return false;
    }
  } /* for(&placement_i..) */
  return true;
} /* cluster_placements_check() */

boost::optional<powerlaw_distributor::cluster_placements> powerlaw_distributor::cluster_placements_calc(
    const cds::const_spatial_entity_vector& entities,
    const rmath::vector3d& block_bb,
    size_t n_clusters) {
  ER_INFO("Computing cluster placements for %zu clusters", n_clusters);

  std::vector<size_t> clust_sizes;

  /* First, calc cluster sizes */
  for (size_t i = 0; i < n_clusters; ++i) {
    /* can't have a cluster of size 0 */
    auto index = static_cast<size_t>(std::max(1U, m_pwrdist(rng())));
    clust_sizes.push_back(index);
  } /* for(i..) */
  auto sum = std::accumulate(std::begin(clust_sizes), std::end(clust_sizes),
                             std::string(),
                             [&](std::string accum, size_t size) {
                               return accum + rcppsw::to_string(size) + ",";
                             });
  ER_DEBUG("Cluster sizes: [%s]", sum.c_str());

  /* Second, place clusters in arena via guess and check */
  for (size_t i = 0; i < kMAX_DIST_TRIES; ++i) {
    auto placements = cluster_placements_guess(clust_sizes, block_bb);
    if (cluster_placements_check(placements, entities)) {
      return boost::make_optional(placements);
    }
  } /* for(i..) */
  ER_FATAL_SENTINEL("Unable to place clusters in arena");
  return boost::none;
} /* cluster_placements_calc() */

ds::block3D_cluster_vector powerlaw_distributor::block_clusters(void) const {
  ds::block3D_cluster_vector ret;

  for (auto& dist : m_dists) {
      auto bclusts = dist->block_clusters();
      ret.insert(ret.end(), bclusts.begin(), bclusts.end());
  }   /* for(i..) */

  return ret;
} /* block_clusters() */

NS_END(block_dist, foraging, cosm);
