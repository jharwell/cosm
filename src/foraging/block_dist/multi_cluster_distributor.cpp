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
    const std::vector<cds::arena_grid::view>& grids,
    cds::arena_grid* arena_grid,
    size_t capacity,
    rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.foraging.block_dist.multi_cluster"),
      base_distributor(arena_grid, rng) {
  for (size_t i = 0; i < grids.size(); ++i) {
    m_dists.emplace_back(rtypes::type_uuid(i),
                         grids[i],
                         arena_grid,
                         capacity,
                         rng);
  } /* for(i..) */
      }
/*******************************************************************************
 * Member Functions
 ******************************************************************************/
dist_status multi_cluster_distributor::distribute_block(
    crepr::base_block3D* block,
    cds::const_spatial_entity_vector& entities) {
  ER_INFO("Distribute block%d: n_clusts=%zu,total_capacity=%zu,total_size=%zu",
          block->id().v(),
          m_dists.size(),
          capacity(),
          size());


  /* -1 because we are working with array indices */
  size_t start = rng()->uniform(0UL, m_dists.size() - 1);

  for (size_t i = 0; i < m_dists.size(); ++i) {
    cluster_distributor& dist = m_dists[(start + i) % m_dists.size()];

    /* Always/only 1 cluster per cluster distributor, so this is safe to do */
    RCSW_UNUSED auto clust_id = dist.block_clusters().front()->id();

    if (dist.capacity() == dist.size()) {
      ER_TRACE("Block%d to cluster%u failed: capacity (%zu) reached",
               block->id().v(),
               clust_id.v(),
               dist.capacity());
      continue;
    }

    ER_DEBUG("Block%d to cluster%u: capacity=%zu,size=%zu",
             block->id().v(),
             clust_id.v(),
             dist.capacity(),
             dist.size());
    double fill = static_cast<double>(dist.size()) /
                  static_cast<double>(dist.capacity());
    if (fill > 0.5) {
      dist.coord_search_policy(coord_search_policy::ekFREE_CELL);
    } else {
      dist.coord_search_policy(coord_search_policy::ekRANDOM);
    }
    auto status = dist.distribute_block(block, entities);
    if (dist_status::ekSUCCESS == status) {
      return status;
    }
  } /* for(i..) */
  return dist_status::ekFAILURE;
} /* distribute_block() */

cfds::block3D_cluster_vector multi_cluster_distributor::block_clusters(void) const {
  cfds::block3D_cluster_vector ret;

  for (auto& dist : m_dists) {
    auto bclusts = dist.block_clusters();
    ret.insert(ret.end(), bclusts.begin(), bclusts.end());
  } /* for(&dist..) */
  return ret;
} /* block_clusters() */

size_t multi_cluster_distributor::size(void) const {
  return std::accumulate(std::begin(m_dists),
                         std::end(m_dists),
                         0,
                         [&](size_t sum, const auto& dist) {
                           return sum + dist.size();
                         });
} /* size() */

NS_END(block_dist, foraging, cosm);
