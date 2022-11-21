/**
 * \file cluster_distributor.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/block_dist/cluster_distributor.hpp"

#include <algorithm>

#include "cosm/ds/cell2D.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::block_dist {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cluster_distributor::cluster_distributor(
    const rtypes::type_uuid& id,
    const cads::arena_grid::view& view,
    cads::arena_grid* arena_grid,
    const cspatial::conflict_checker::map_cb_type& conflict_check,
    const random_distributor::dist_success_cb_type& dist_success,
    size_t capacity,
    rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.foraging.block_dist.cluster"),
      base_distributor(arena_grid, rng),
      m_clust(id, view, arena_grid->resolution(), capacity),
      m_impl(view, arena_grid, conflict_check, dist_success, rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
dist_status cluster_distributor::distribute_block(crepr::sim_block3D* block) {
  if (m_clust.capacity() == m_clust.n_blocks()) {
    ER_DEBUG("Could not distribute block%d: Cluster capacity (%zu) reached",
             block->id().v(),
             m_clust.capacity());
    return dist_status::ekFAILURE;
  }
  ER_DEBUG("Distribute block to cluster%d, capacity=%zu,size=%zu",
           m_clust.id().v(),
           m_clust.capacity(),
           m_clust.n_blocks());

  /* do distribution */
  auto status = m_impl.distribute_block(block);

  if (dist_status::ekSUCCESS == status) {
    /* update block cluster--the distributed block is now in it */
    m_clust.update_after_drop(block);
  }

  return status;
} /* distribute_block() */

dist_status cluster_distributor::distribute_blocks(cds::block3D_vectorno& blocks,
                                                   bool strict_success) {
  if (m_clust.capacity() == m_clust.n_blocks()) {
    ER_DEBUG("Could not distribute any of %zu blocks: Cluster capacity (%zu) "
             "reached",
             blocks.size(),
             m_clust.capacity());
    return dist_status::ekFAILURE;
  }
  auto status = m_impl.distribute_blocks(blocks, strict_success);

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
  return { &m_clust };
} /* block_clusters() */

} /* namespace cosm::foraging::block_dist */
