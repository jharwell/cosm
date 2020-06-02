/**
 * \file cluster_distributor.cpp
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
#include "cosm/foraging/block_dist/cluster_distributor.hpp"

#include <algorithm>

#include "cosm/ds/cell2D.hpp"
#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cluster_distributor::cluster_distributor(
    const cds::arena_grid::view& view,
    const rtypes::discretize_ratio& resolution,
    uint capacity,
    rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.foraging.block_dist.cluster"),
      base_distributor(rng),
      m_clust(view, resolution, capacity),
      m_impl(view, resolution, rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool cluster_distributor::distribute_block(crepr::base_block3D* block,
                                           cds::const_entity_vector& entities) {
  if (m_clust.capacity() == m_clust.block_count()) {
    ER_DEBUG("Could not distribute block%d: Cluster capacity (%u) reached",
             block->id().v(),
             m_clust.capacity());
    return false;
  }
  return m_impl.distribute_block(block, entities);
} /* distribute_block() */

bool cluster_distributor::distribute_blocks(cds::block3D_vectorno& blocks,
                                            cds::const_entity_vector& entities) {
  if (m_clust.capacity() == m_clust.block_count()) {
    ER_DEBUG(
        "Could not distribute any of %zu blocks: Cluster capacity (%u) reached",
        blocks.size(),
        m_clust.capacity());
    return false;
  }
  return m_impl.distribute_blocks(blocks, entities);
} /* distribute_blocks() */

cfds::block3D_cluster_vector cluster_distributor::block_clusters(void) const {
  return cfds::block3D_cluster_vector{&m_clust};
} /* block_clusters() */

NS_END(block_dist, foraging, cosm);
