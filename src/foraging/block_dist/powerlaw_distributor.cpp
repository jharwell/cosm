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

#include <map>

#include "cosm/foraging/block_dist/multi_cluster_distributor.hpp"
#include "cosm/foraging/block_dist/powerlaw_cluster_placer.hpp"
#include "cosm/foraging/config/block_dist_config.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);
using cosm::arena::ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
powerlaw_distributor::powerlaw_distributor(
    const config::powerlaw_dist_config* const config,
    cads::arena_grid* grid,
    rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.foraging.block_dist.powerlaw"),
      base_distributor(grid, rng),
      m_config_clusters(config->n_clusters),
      m_pwrdist(config->pwr_min, config->pwr_max, 2) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
dist_status powerlaw_distributor::distribute_block(crepr::sim_block3D* block) {
  /*
   * Try to find a block cluster to distribute to, starting from a random
   * cluster size.
   */
  size_t start = rng()->uniform(rmath::rangez(0, m_dists.size() - 1));
  for (size_t i = 0; i < m_dists.size(); ++i) {
    auto* mclust = m_dists[(start + i) % m_dists.size()].get();

    if (mclust->size() == mclust->capacity()) {
      continue;
    }
    ER_INFO("Attempt distribution: block%d -> cluster group: "
             "capacity=%zu,size=%zu]",
             block->id().v(),
             mclust->capacity(),
             mclust->size());

    if (dist_status::ekSUCCESS == mclust->distribute_block(block)) {
      return dist_status::ekSUCCESS;
    }
  } /* for(i..) */

  return dist_status::ekFAILURE;
} /* distribute_block() */

void powerlaw_distributor::initialize(
    carena::base_arena_map* map,
    const cds::const_spatial_entity_vector& c_entities,
    const rmath::vector3d& c_block_bb,
    const cspatial::conflict_checker::map_cb_type& conflict_check,
    const base_distributor::dist_success_cb_type& dist_success) {
  std::vector<size_t> clust_sizes;

  /* First, calc cluster sizes, and sort */
  for (size_t i = 0; i < m_config_clusters; ++i) {
    /* can't have a cluster of size 0 */
    auto index = std::max(static_cast<decltype(m_pwrdist(rng()))>(1),
                          m_pwrdist(rng()));
    clust_sizes.push_back(index);
  } /* for(i..) */
  std::sort(clust_sizes.begin(), clust_sizes.end(), std::greater<>());

  auto sum = std::accumulate(std::begin(clust_sizes),
                             std::end(clust_sizes),
                             std::string(),
                             [&](std::string accum, size_t size) {
                               return accum + rcppsw::to_string(size) + ",";
                             });
  ER_DEBUG("Cluster sizes: [%s]", sum.c_str());

  /* Compute cluster locations in arena */
  powerlaw_cluster_placer placer(
      map, c_block_bb, kMAX_DIST_TRIES, rng());
  auto placements = placer(c_entities, clust_sizes);

  std::map<size_t, std::vector<cads::arena_grid::view>> grids;
  std::for_each(placements.begin(), placements.end(), [&](const auto& placement) {
    return grids[placement.capacity].push_back(placement.view);
  });

  size_t id_start = 0;
  for (auto& pair : grids) {
    auto mclust = std::make_unique<multi_cluster_distributor>(
        pair.second,
        arena_grid(),
        conflict_check,
        dist_success,
        pair.first,
        rtypes::type_uuid(id_start),
        rng());
    ER_INFO("Mapped multi-cluster: capacity=%zu,ID start=%zu",
            mclust->capacity(),
            id_start);
    m_dists.push_back(std::move(mclust));
    id_start += pair.second.size();
  } /* for(i..) */
} /* initialize() */

ds::block3D_cluster_vectorno powerlaw_distributor::block_clustersno(void) {
  ds::block3D_cluster_vectorno ret;

  for (auto& dist : m_dists) {
    auto bclusts = dist->block_clustersno();
    ret.insert(ret.end(), bclusts.begin(), bclusts.end());
  } /* for(i..) */

  return ret;
} /* block_clustersno() */

size_t powerlaw_distributor::capacity(void) const {
  return std::accumulate(
      std::begin(m_dists), std::end(m_dists), 0, [&](size_t size, const auto& d) {
        return size + d->capacity();
      });
} /* capacity() */

size_t powerlaw_distributor::size(void) const {
  return std::accumulate(
      std::begin(m_dists), std::end(m_dists), 0, [&](size_t size, const auto& d) {
        return size + d->size();
      });
} /* capacity() */

NS_END(block_dist, foraging, cosm);
