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
cluster_distributor::cluster_distributor(const rtypes::type_uuid& id,
                                         const cds::arena_grid::view& view,
                                         cds::arena_grid* arena_grid,
                                         size_t capacity,
                                         rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.foraging.block_dist.cluster"),
      base_distributor(arena_grid, rng),
      m_clust(id, view, arena_grid->resolution(), capacity),
      m_impl(view, arena_grid, rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
dist_status cluster_distributor::distribute_block(
    crepr::base_block3D* block,
    cds::const_spatial_entity_vector& entities) {
  if (m_clust.capacity() == m_clust.n_blocks()) {
    ER_DEBUG("Could not distribute block%d: Cluster capacity (%zu) reached",
             block->id().v(),
             m_clust.capacity());
    return dist_status::ekFAILURE;
  }
  /* do distribution */
  auto status = m_impl.distribute_block(block, entities);

  if (dist_status::ekSUCCESS == status) {
    /* update block cluster--the distributed block is now in it */
    m_clust.update_after_drop(block);
  }

  return status;
} /* distribute_block() */

dist_status
cluster_distributor::distribute_blocks(cds::block3D_vectorno& blocks,
                                       cds::const_spatial_entity_vector& entities,
                                       bool strict_success) {
  if (m_clust.capacity() == m_clust.n_blocks()) {
    ER_DEBUG("Could not distribute any of %zu blocks: Cluster capacity (%zu) "
             "reached",
             blocks.size(),
             m_clust.capacity());
    return dist_status::ekFAILURE;
  }
  auto status = m_impl.distribute_blocks(blocks, entities, strict_success);

  /*
   * If strict_success is TRUE, then if for some reason not all blocks can be
   * distributed then status will be ekFAILURE, even though SOME blocks probably
   * were successfully distributed, and we need to make sure we build the
   * original block set correctly for our cluster.
   */
  if ((dist_status::ekSUCCESS == status) || strict_success) {
    /* update block cluster */
    m_clust.blocks_recalc();
  }
  return status;
} /* distribute_blocks() */

cfds::block3D_cluster_vectorno cluster_distributor::block_clustersno(void) {
  return cfds::block3D_cluster_vectorno{ &m_clust };
} /* block_clusters() */

NS_END(block_dist, foraging, cosm);
