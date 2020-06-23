/**
 * \file multi_cluster_distributor.cpp
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
#include "cosm/foraging/block_dist/multi_cluster_distributor.hpp"

#include "cosm/ds/cell2D.hpp"
#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
multi_cluster_distributor::multi_cluster_distributor(
    std::vector<cds::arena_grid::view>& grids,
    rtypes::discretize_ratio resolution,
    uint maxsize,
    rmath::rng* rng_in)
    : ER_CLIENT_INIT("cosm.foraging.block_dist.multi_cluster"),
      base_distributor(rng_in) {
  for (auto& g : grids) {
    m_dists.emplace_back(g, resolution, maxsize, rng_in);
  } /* for(i..) */
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool multi_cluster_distributor::distribute_block(
    crepr::base_block3D* block,
    cds::const_spatial_entity_vector& entities) {
  for (uint i = 0; i < kMAX_DIST_TRIES; ++i) {
    /* -1 because we are working with array indices */
    uint idx = rng()->uniform(0, m_dists.size() - 1);
    cluster_distributor& dist = m_dists[idx];

    /* Always/only 1 cluster per cluster distributor, so this is safe to do */
    auto* clust = dist.block_clusters().front();
    if (clust->capacity() == clust->block_count()) {
      ER_DEBUG("Block%d to cluster%u failed: capacity (%u) reached",
               block->id().v(),
               idx,
               clust->capacity());
    } else {
      ER_DEBUG("Block%d to cluster%u: capacity=%u,size=%zu",
               block->id().v(),
               idx,
               clust->capacity(),
               clust->block_count());
      return dist.distribute_block(block, entities);
    }
  } /* for(i..) */
  return false;
} /* distribute_block() */

cfds::block3D_cluster_vector multi_cluster_distributor::block_clusters(
    void) const {
  cfds::block3D_cluster_vector ret;

  for (auto& dist : m_dists) {
    auto bclusts = dist.block_clusters();
    ret.insert(ret.end(), bclusts.begin(), bclusts.end());
  } /* for(&dist..) */
  return ret;
} /* block_clusters() */

NS_END(block_dist, foraging, cosm);
