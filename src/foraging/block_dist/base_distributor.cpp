/**
 * \file base_distributor.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/block_dist/base_distributor.hpp"

#include "cosm/foraging/repr/block_cluster.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_distributor::clusters_update(void) {
  for (auto* clust : block_clustersno()) {
    clust->blocks_recalc();
  } /* for(*clust..) */
}

void base_distributor::cluster_update_after_pickup(
    const crepr::sim_block3D* const block,
    const rmath::vector2z& old_loc) {
  auto clusters = block_clustersno();
  for (auto* clust : clusters) {
    if (clust->contains_abs(old_loc)) {
      clust->update_after_pickup(block->id());
      return;
    }
  } /* for(*clust..) */
} /* cluster_update_after_pickup() */

void base_distributor::cluster_update_after_drop(
    const crepr::sim_block3D* const block) {
  auto clusters = block_clustersno();
  for (auto* clust : clusters) {
    if (clust->contains_abs(block->danchor2D())) {
      clust->update_after_drop(block);
      return;
    }
  } /* for(*clust..) */
} /* cluster_update_after_drop() */

cfds::block3D_cluster_vectorro base_distributor::block_clustersro(void) const {
  auto clusters = const_cast<base_distributor*>(this)->block_clustersno();
  cfds::block3D_cluster_vectorro ret(clusters.size());
  std::transform(clusters.begin(),
                 clusters.end(),
                 ret.begin(),
                 [&](const auto& clust) { return clust; });
  return ret;
}

NS_END(block_dist, foraging, cosm);
