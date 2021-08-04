/**
 * \file base_distributor.cpp
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
#include "cosm/foraging/block_dist/base_distributor.hpp"

#include "cosm/foraging/repr/block_cluster.hpp"
#include "cosm/repr/base_block3D.hpp"

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
  for (auto *clust : block_clustersno()) {
    clust->blocks_recalc();
  } /* for(*clust..) */
}

void base_distributor::cluster_update_after_pickup(const crepr::base_block3D* const block,
                                                   const rmath::vector2z& old_loc) {
  auto clusters = block_clustersno();
  for (auto *clust : clusters) {
    if (clust->contains_abs(old_loc)) {
      clust->update_after_pickup(block->id());
      return;
    }
  } /* for(*clust..) */
} /* cluster_update_after_pickup() */

void base_distributor::cluster_update_after_drop(const crepr::base_block3D* const block) {
  auto clusters = block_clustersno();
  for (auto *clust : clusters) {
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
